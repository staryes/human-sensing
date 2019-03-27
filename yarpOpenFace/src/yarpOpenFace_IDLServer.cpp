/*
 * Copyright (C) 2006-2019 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

// This is an automatically generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#include <yarpOpenFace_IDLServer.h>
#include <yarp/os/idl/WireTypes.h>



class yarpOpenFace_IDLServer_display : public yarp::os::Portable {
public:
  std::string element;
  std::string value;
  bool _return;
  void init(const std::string& element, const std::string& value);
  bool write(yarp::os::ConnectionWriter& connection) const override;
  bool read(yarp::os::ConnectionReader& connection) override;
};

class yarpOpenFace_IDLServer_quit : public yarp::os::Portable {
public:
  bool _return;
  void init();
  bool write(yarp::os::ConnectionWriter& connection) const override;
  bool read(yarp::os::ConnectionReader& connection) override;
};

bool yarpOpenFace_IDLServer_display::write(yarp::os::ConnectionWriter& connection) const {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(3)) return false;
  if (!writer.writeTag("display",1,1)) return false;
  if (!writer.writeString(element)) return false;
  if (!writer.writeString(value)) return false;
  return true;
}

bool yarpOpenFace_IDLServer_display::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void yarpOpenFace_IDLServer_display::init(const std::string& element, const std::string& value) {
  _return = false;
  this->element = element;
  this->value = value;
}

bool yarpOpenFace_IDLServer_quit::write(yarp::os::ConnectionWriter& connection) const {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("quit",1,1)) return false;
  return true;
}

bool yarpOpenFace_IDLServer_quit::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void yarpOpenFace_IDLServer_quit::init() {
  _return = false;
}

yarpOpenFace_IDLServer::yarpOpenFace_IDLServer() {
  yarp().setOwner(*this);
}
bool yarpOpenFace_IDLServer::display(const std::string& element, const std::string& value) {
  bool _return = false;
  yarpOpenFace_IDLServer_display helper;
  helper.init(element,value);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool yarpOpenFace_IDLServer::display(const std::string& element, const std::string& value)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool yarpOpenFace_IDLServer::quit() {
  bool _return = false;
  yarpOpenFace_IDLServer_quit helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool yarpOpenFace_IDLServer::quit()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}

bool yarpOpenFace_IDLServer::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  reader.expectAccept();
  if (!reader.readListHeader()) { reader.fail(); return false; }
  std::string tag = reader.readTag();
  bool direct = (tag=="__direct__");
  if (direct) tag = reader.readTag();
  while (!reader.isError()) {
    // TODO: use quick lookup, this is just a test
    if (tag == "display") {
      std::string element;
      std::string value;
      if (!reader.readString(element)) {
        reader.fail();
        return false;
      }
      if (!reader.readString(value)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = display(element,value);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "quit") {
      bool _return;
      _return = quit();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "help") {
      std::string functionName;
      if (!reader.readString(functionName)) {
        functionName = "--all";
      }
      std::vector<std::string> _return=help(functionName);
      yarp::os::idl::WireWriter writer(reader);
        if (!writer.isNull()) {
          if (!writer.writeListHeader(2)) return false;
          if (!writer.writeTag("many",1, 0)) return false;
          if (!writer.writeListBegin(BOTTLE_TAG_INT32, static_cast<uint32_t>(_return.size()))) return false;
          std::vector<std::string> ::iterator _iterHelp;
          for (_iterHelp = _return.begin(); _iterHelp != _return.end(); ++_iterHelp)
          {
            if (!writer.writeString(*_iterHelp)) return false;
           }
          if (!writer.writeListEnd()) return false;
        }
      reader.accept();
      return true;
    }
    if (reader.noMore()) { reader.fail(); return false; }
    std::string next_tag = reader.readTag();
    if (next_tag=="") break;
    tag.append("_").append(next_tag);
  }
  return false;
}

std::vector<std::string> yarpOpenFace_IDLServer::help(const std::string& functionName) {
  bool showAll=(functionName=="--all");
  std::vector<std::string> helpString;
  if(showAll) {
    helpString.emplace_back("*** Available commands:");
    helpString.emplace_back("display");
    helpString.emplace_back("quit");
    helpString.emplace_back("help");
  }
  else {
    if (functionName=="display") {
      helpString.emplace_back("bool display(const std::string& element, const std::string& value) ");
      helpString.emplace_back("* Selects what to draw (defaults landmarks on) ");
      helpString.emplace_back("* @param element specifies which element is requested (landmarks | points | labels | dark-mode) ");
      helpString.emplace_back("@param value specifies its value (on | off) ");
      helpString.emplace_back("* @return true/false on success/failure ");
    }
    if (functionName=="quit") {
      helpString.emplace_back("bool quit() ");
      helpString.emplace_back("Quit the module. ");
      helpString.emplace_back("@return true/false on success/failure ");
    }
    if (functionName=="help") {
      helpString.emplace_back("std::vector<std::string> help(const std::string& functionName=\"--all\")");
      helpString.emplace_back("Return list of available commands, or help message for a specific function");
      helpString.emplace_back("@param functionName name of command for which to get a detailed description. If none or '--all' is provided, print list of available commands");
      helpString.emplace_back("@return list of strings (one string per line)");
    }
  }
  if ( helpString.empty()) helpString.emplace_back("Command not found");
  return helpString;
}


