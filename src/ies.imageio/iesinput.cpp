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
#include <OpenEXR/ImathVec.h>

#include <cstdio>
#include <cstdlib>
#include <map>
#include <cstring>

#include "imageio.h"
#include "ies_pvt.h"
#include "photometric.h"
#include "filesystem.h"

OIIO_PLUGIN_NAMESPACE_BEGIN


using namespace pvt;

class IESInput:public IESInput_Interface {
public:
	IESInput() {}
	virtual ~IESInput() {}
	virtual const char * format_name(void) const { return "ies"; }
	virtual bool valid_file(const std::string &filename) const;
	virtual bool open(const std::string & name, ImageSpec & newSpec);
	virtual bool sample(const Imath::V3f & dir, float & res);
	virtual bool close();
	virtual bool read_native_scanline(int y, int z, void *data);
private:
	PhotometricSampler  mSampler;

	void reset()
	{
		mSampler.clear();
	}
};

OIIO_PLUGIN_EXPORTS_BEGIN

OIIO_EXPORT ImageInput *
ies_input_imageio_create()
{
	return new IESInput;
}

OIIO_EXPORT const char * ies_input_extensions[] = {
	"ies", NULL
};

OIIO_PLUGIN_EXPORTS_END

bool
IESInput::valid_file(const std::string &filename) const
{
	if (!Filesystem::is_regular(filename)) {
		return false;
	}
	
	PhotometricDataIES ies(filename.c_str());

	bool ok = ies.isValid();
	
	return ok;
}

bool
IESInput::open(const std::string & name, ImageSpec & newSpec)
{
	reset();

	if (!Filesystem::is_regular(name)) {
		return false;
	}

	mSampler.load(name.c_str(), false);

	bool ok = mSampler.is_valid();

	return ok;
}

bool
IESInput::close()
{
	reset();
	return true;
}

bool
IESInput::read_native_scanline(int y, int z, void *data)
{
	// scanlines not supported
	return false;
}

bool 
IESInput::sample(const Imath::V3f & dir, float& res)
{
	res = mSampler.sample(dir);

	return true;
}

OIIO_PLUGIN_NAMESPACE_END
