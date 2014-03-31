/*
Copyright 2010 Larry Gritz and the other authors and contributors.
All Rights Reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:
* Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.
* Neither the name of the software's owners nor the names of its
contributors may be used to endorse or promote products derived from
this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

(This is the Modified BSD License)
*/
#include <boost/shared_ptr.hpp>
#include <openvdb/openvdb.h>
#include <openvdb/tools/Interpolation.h>
#include <OpenEXR/ImathVec.h>
#include <boost/unordered_map.hpp>

#include <cstdio>
#include <cstdlib>



#include "dassert.h"
#include "imageio.h"
#include "thread.h"
#include "ustring.h"

#include "openvdb_pvt.h"
#include "filesystem.h"

OIIO_PLUGIN_NAMESPACE_BEGIN

namespace pvt {
	spin_mutex &openvdb_mutex() {
		static spin_mutex m;
		return m;
	}
};

using namespace pvt;

class BaseTypeReader {
public:
	BaseTypeReader() {};
	virtual ~BaseTypeReader() {};
	virtual void lookup_voxel(const Imath::V3d& wsP, void * aData, int thread_index) = 0;
};

template<typename GridType>
class DataTypeReader : public BaseTypeReader {
public:
	typedef typename GridType::TreeType GridTreeType;
	typedef typename GridType::ValueType DataType;
	typedef typename GridType::Ptr GridPtr;
	typedef typename GridType::ConstAccessor AccessorType;
	typedef boost::shared_ptr<AccessorType> AccPtr;
	typedef openvdb::tools::GridSampler<AccessorType, openvdb::tools::BoxSampler> GridSamplerType;	

	DataTypeReader() {};
	~DataTypeReader() {};
	
	void init(GridPtr aGrid)
	{
		mGrid = aGrid;
		mBBox = worldSpaceBBox(mGrid->transform(), mGrid->evalActiveVoxelBoundingBox());
	}

	void lookup_voxel(const Imath::V3d& wsP, void * aData, int thread_index)
	{
		if (!mAccVec[thread_index])
		{	
			mAccVec[thread_index] = boost::shared_ptr<AccessorType>(new AccessorType(mGrid->getConstAccessor()));
		}
	
		GridSamplerType mSampler(*mAccVec[thread_index], mGrid->constTransform());
		openvdb::math::Vec3d V(wsP.x, wsP.y, wsP.z);

		// we need to transform the vector to the Vdb world space
		openvdb::math::Vec3d vMin = mBBox.min();
		openvdb::math::Vec3d vMax = mBBox.max();
		openvdb::math::Vec3d vW = V * (vMax - vMin) + vMin;
		*((DataType*)aData)= mSampler.wsSample(vW);
	}

private:
	AccPtr mAccVec[VDB_MAX_NUM_THREADS]; // the vector of accessor for every threads
	GridPtr mGrid;
	openvdb::BBoxd mBBox;
};

typedef boost::shared_ptr<BaseTypeReader> ReaderRef;
typedef boost::unordered::unordered_map<ustring, ReaderRef> GridMap;

class OpenVDBInput:public OpenVDBInput_Interface{
public:
	OpenVDBInput()
	{
		reset();
	}

	virtual ~OpenVDBInput() {}
	virtual const char * format_name(void) const { return "vdb"; }
	virtual bool valid_file(const std::string &filename) const;
	virtual bool open(const std::string &name, ImageSpec & newspec);
	virtual bool lookup_data(const Imath::V3d& wsP, ustring aGridName, float * aData, int actualchannels, int thread_index);
	virtual bool close();
	virtual bool read_native_scanline(int y, int z, void *data);

private:
	std::string m_name;
	boost::shared_ptr<openvdb::io::File> m_input;
	openvdb::GridPtrVecPtr mGridVec; // all grids in a file
	GridMap mGridMap; // the map list of every grid in a VDB file 
	
	void reset()
	{
		m_name.clear();
		m_input = boost::shared_ptr<openvdb::io::File>();
		mGridVec = openvdb::GridPtrVecPtr();
		mGridMap = GridMap();
	}

	bool read_all_grids();
};

OIIO_PLUGIN_EXPORTS_BEGIN

OIIO_EXPORT ImageInput *
openvdb_input_imageio_create()
{
	return new OpenVDBInput;
}

OIIO_EXPORT const char * openvdb_input_extensions[] = {
	"vdb",NULL
};

OIIO_PLUGIN_EXPORTS_END

void
oiio_openvdb_initialize()
{
	openvdb::initialize(); //OpenVDB is thread safe 
}

bool
OpenVDBInput::valid_file(const std::string &filename) const
{
	if (!Filesystem::is_regular(filename))
		return false;

	oiio_openvdb_initialize(); // call initialize multiple times is ok because vdb has already handled it
	
	bool ok = false;
	
	openvdb::io::File input(filename);
	{
		spin_lock lock(openvdb_mutex());
		try {
			ok = input.open();
		} catch (...) {
			ok = false;
		}
	}

	return ok;
}

bool
OpenVDBInput::open(const std::string &name, ImageSpec &newspec)
{

	if (m_input)
		close();
	
	if (!Filesystem::is_regular(name))
		return false;

	oiio_openvdb_initialize();	
	
	spin_lock lock(openvdb_mutex());
	
	m_input = boost::shared_ptr<openvdb::io::File>(new openvdb::io::File(name));
	
	bool ok = false;
	try {
		ok = m_input->open();
	} catch (...) {
		ok = false;
	}

	if (!ok) {
		m_input = boost::shared_ptr<openvdb::io::File>();
		m_name.clear();
		return false;
	}

	if (!read_all_grids())
		return false;

	for (int i = 0; i < mGridVec->size(); ++i)
	{
		const std::string & tname = (*mGridVec)[i]->getName();
		ustring name = ustring(tname);
		const std::string & ttype = (*mGridVec)[i]->valueType();

		if (ttype == "float")
		{
			openvdb::FloatGrid::Ptr tGrid = openvdb::gridPtrCast<openvdb::FloatGrid>((*mGridVec)[i]);
			DataTypeReader<openvdb::FloatGrid> * tReader = new DataTypeReader<openvdb::FloatGrid>;
			tReader->init(tGrid);
			ReaderRef rRef = ReaderRef(tReader);
			mGridMap.insert(GridMap::value_type(name,rRef));
		}
		else if (ttype == "vec3s")
		{
			openvdb::Vec3SGrid::Ptr tGrid = openvdb::gridPtrCast<openvdb::Vec3SGrid>((*mGridVec)[i]);
			DataTypeReader<openvdb::Vec3SGrid> * tReader = new DataTypeReader<openvdb::Vec3SGrid>;
			tReader->init(tGrid);
			ReaderRef rRef = ReaderRef(tReader);
			mGridMap.insert(GridMap::value_type(name,rRef));

		}
		else if (ttype == "int32_t")
		{
			openvdb::Int32Grid::Ptr tGrid = openvdb::gridPtrCast<openvdb::Int32Grid>((*mGridVec)[i]);
			DataTypeReader<openvdb::Int32Grid> * tReader = new DataTypeReader<openvdb::Int32Grid>;
			tReader->init(tGrid);
			ReaderRef rRef = ReaderRef(tReader);
			mGridMap.insert(GridMap::value_type(name,rRef));
		}
		else if (ttype == "bool")
		{
			openvdb::BoolGrid::Ptr tGrid = openvdb::gridPtrCast<openvdb::BoolGrid>((*mGridVec)[i]);
			DataTypeReader<openvdb::BoolGrid> * tReader = new DataTypeReader<openvdb::BoolGrid>;
			tReader->init(tGrid);
			ReaderRef rRef = ReaderRef(tReader);
			mGridMap.insert(GridMap::value_type(name,rRef));
		}
		// we don't care about other types for now
	}

	m_name = name;
	
	return true;
}

bool 
OpenVDBInput::close()
{
	spin_lock lock(openvdb_mutex());
	
	if (m_input) {
		m_input->close();
	}
	
	reset();

	return true;
}

bool
OpenVDBInput::read_native_scanline(int y, int z, void *data)
{
	// scanlines not supported
	return false;
}

bool
OpenVDBInput::read_all_grids()
{
	if (!m_input)
		return false;
	
	mGridVec = m_input->getGrids();
	
	if (mGridVec->empty()){
		OPENVDB_LOG_WARN(m_name << " is empty");
		return false;
	}

	return true;
}

/// Transform the Bbox of coordinates to worldspace
openvdb::BBoxd
worldSpaceBBox(const openvdb::math::Transform& xform, const openvdb::CoordBBox& bbox)
{
	const double MAX_DOUBLE = std::numeric_limits<double>::max();

	openvdb::Vec3d pMin = openvdb::Vec3d(MAX_DOUBLE, MAX_DOUBLE, MAX_DOUBLE);
	openvdb::Vec3d pMax = -pMin;

	const openvdb::Coord& min = bbox.min();
	const openvdb::Coord& max = bbox.max();
	openvdb::Coord ijk;

	// corner 1
	openvdb::Vec3d ptn = xform.indexToWorld(min);
	for (int i = 0; i < 3; ++i) {
		if (ptn[i] < pMin[i]) pMin[i] = ptn[i];
		if (ptn[i] > pMax[i]) pMax[i] = ptn[i];
	}

	// corner 2
	ijk[0] = min.x();
	ijk[1] = min.y();
	ijk[2] = max.z();
	ptn = xform.indexToWorld(ijk);
	for (int i = 0; i < 3; ++i) {
		if (ptn[i] < pMin[i]) pMin[i] = ptn[i];
		if (ptn[i] > pMax[i]) pMax[i] = ptn[i];
	}

	// corner 3
	ijk[0] = max.x();
	ijk[1] = min.y();
	ijk[2] = max.z();
	ptn = xform.indexToWorld(ijk);
	for (int i = 0; i < 3; ++i) {
		if (ptn[i] < pMin[i]) pMin[i] = ptn[i];
		if (ptn[i] > pMax[i]) pMax[i] = ptn[i];
	}

	// corner 4
	ijk[0] = max.x();
	ijk[1] = min.y();
	ijk[2] = min.z();
	ptn = xform.indexToWorld(ijk);
	for (int i = 0; i < 3; ++i) {
		if (ptn[i] < pMin[i]) pMin[i] = ptn[i];
		if (ptn[i] > pMax[i]) pMax[i] = ptn[i];
	}

	// corner 5
	ijk[0] = min.x();
	ijk[1] = max.y();
	ijk[2] = min.z();
	ptn = xform.indexToWorld(ijk);
	for (int i = 0; i < 3; ++i) {
		if (ptn[i] < pMin[i]) pMin[i] = ptn[i];
		if (ptn[i] > pMax[i]) pMax[i] = ptn[i];
	}

	// corner 6
	ijk[0] = min.x();
	ijk[1] = max.y();
	ijk[2] = max.z();
	ptn = xform.indexToWorld(ijk);
	for (int i = 0; i < 3; ++i) {
		if (ptn[i] < pMin[i]) pMin[i] = ptn[i];
		if (ptn[i] > pMax[i]) pMax[i] = ptn[i];
	}

	// corner 7
	ptn = xform.indexToWorld(max);
	for (int i = 0; i < 3; ++i) {
		if (ptn[i] < pMin[i]) pMin[i] = ptn[i];
		if (ptn[i] > pMax[i]) pMax[i] = ptn[i];
	}

	// corner 8
	ijk[0] = max.x();
	ijk[1] = max.y();
	ijk[2] = min.z();
	ptn = xform.indexToWorld(ijk);
	for (int i = 0; i < 3; ++i) {
		if (ptn[i] < pMin[i]) pMin[i] = ptn[i];
		if (ptn[i] > pMax[i]) pMax[i] = ptn[i];
	}

	return openvdb::BBoxd(pMin, pMax);
}

bool
OpenVDBInput::lookup_data(const Imath::V3d& wsP, ustring aGridName, float * aData, int actualchannels, int thread_index)
{
	GridMap::const_iterator gridit = mGridMap.find(aGridName);
	
	if (gridit != mGridMap.end())
	{
		float dataRes[4];
		
		gridit->second.get()->lookup_voxel(wsP, (void *)dataRes, thread_index);

		for (int i = 0; i < actualchannels; ++i)
		{
			aData[i] = ((float *)dataRes)[i];
		}
		
		return true;
	}

	return false;
}

OIIO_PLUGIN_NAMESPACE_END
