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
#include <openvdb/openvdb.h>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <iostream>

#include "dassert.h"
#include "imageio.h"
#include "strutil.h"
#include "sysutil.h"


OIIO_PLUGIN_NAMESPACE_BEGIN


class OpenVDBOutput : public ImageOutput {
public:
    OpenVDBOutput ();
    virtual ~OpenVDBOutput ();
    virtual const char * format_name (void) const { return "vdb"; }
    virtual bool supports (const std::string &feature) const;
    virtual bool open (const std::string &name, const ImageSpec &spec, OpenMode mode);
    virtual bool close ();
    virtual bool write_scanline (int y, int z, TypeDesc format,
                                 const void *data, stride_t xstride);
    virtual bool write_tile (int x, int y, int z,
                             TypeDesc format, const void *data,
                             stride_t xstride, stride_t ystride, stride_t zstride);

private:
    // Initialize private members to pre-opened state
    void init (void) {
    }

    // Add a parameter to the output
    bool put_parameter (const std::string &name, TypeDesc type,
                        const void *data);
};




// Obligatory material to make this a recognizeable imageio plugin:
OIIO_PLUGIN_EXPORTS_BEGIN

OIIO_EXPORT ImageOutput *
openvdb_output_imageio_create ()
{
    return new OpenVDBOutput;
}

OIIO_EXPORT int openvdb_imageio_version = OIIO_PLUGIN_VERSION;

OIIO_EXPORT const char * openvdb_output_extensions[] = {
    "vdb", NULL
};

OIIO_PLUGIN_EXPORTS_END



OpenVDBOutput::OpenVDBOutput ()
{
    init ();
}



OpenVDBOutput::~OpenVDBOutput ()
{
    // Close, if not already done.
    close ();
}



bool
OpenVDBOutput::supports (const std::string &feature) const
{
    return false;
}



bool
OpenVDBOutput::open (const std::string &name, const ImageSpec &userspec, OpenMode mode)
{
    return false;
}



bool
OpenVDBOutput::put_parameter (const std::string &name, TypeDesc type,
                              const void *data)
{
    return false;
}



bool
OpenVDBOutput::close ()
{

    init ();      // re-initialize
    return true;  // How can we fail?
}



bool
OpenVDBOutput::write_scanline (int y, int z, TypeDesc format,
                               const void *data, stride_t xstride)
{
    return false;
}



bool
OpenVDBOutput::write_tile (int x, int y, int z,
                           TypeDesc format, const void *data,
                           stride_t xstride, stride_t ystride, stride_t zstride)
{
    return false;
}


OIIO_PLUGIN_NAMESPACE_END

