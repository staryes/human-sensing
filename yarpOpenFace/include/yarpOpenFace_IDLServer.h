/*
 * Copyright (C) 2006-2019 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

// This is an automatically generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_yarpOpenFace_IDLServer
#define YARP_THRIFT_GENERATOR_yarpOpenFace_IDLServer

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>

class yarpOpenFace_IDLServer;


/**
 * yarpOpenFace_IDLServer
 * Interface.
 */
class yarpOpenFace_IDLServer : public yarp::os::Wire {
public:
  yarpOpenFace_IDLServer();
  /**
   * * Selects what to draw (defaults landmarks on)
   * * @param element specifies which element is requested (landmarks | points | labels | dark-mode)
   * @param value specifies its value (on | off)
   * * @return true/false on success/failure
   */
  virtual bool display(const std::string& element, const std::string& value);
  /**
   * Quit the module.
   * @return true/false on success/failure
   */
  virtual bool quit();
  bool read(yarp::os::ConnectionReader& connection) override;
  virtual std::vector<std::string> help(const std::string& functionName="--all");
};

#endif
