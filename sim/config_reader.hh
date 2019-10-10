// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Copyright (C) 2019 CAMELab
 *
 * Author: Donghyun Gouk <kukdh1@camelab.org>
 */

#pragma once

#ifndef __SIMPLESSD_SIM_CONFIG_READER_HH__
#define __SIMPLESSD_SIM_CONFIG_READER_HH__

#include <string>

#include "cpu/config.hh"
#include "hil/config.hh"
#include "mem/config.hh"
#include "sim/config.hh"

namespace SimpleSSD {

//! Configuration section enum.
enum class Section {
  Simulation,
  CPU,
  Memory,
  HostInterface,
};

/**
 * \brief ConfigReader object declaration
 *
 * SSD configuration object. This object provides configuration parser.
 * Also, you can override configuration by calling set function.
 */
class ConfigReader {
 private:
  pugi::xml_document file;

  Config simConfig;
  CPU::Config cpuConfig;
  Memory::Config memConfig;
  HIL::Config hilConfig;

 public:
  ConfigReader();
  ConfigReader(const ConfigReader &) = delete;
  ConfigReader(ConfigReader &&) noexcept = default;
  ~ConfigReader();

  ConfigReader &operator=(const ConfigReader &) = delete;
  ConfigReader &operator=(ConfigReader &&) = default;

  void load(const char *) noexcept;
  void load(std::string &) noexcept;

  void save(const char *) noexcept;
  void save(std::string &) noexcept;

  int64_t readInt(Section, uint32_t);
  uint64_t readUint(Section, uint32_t);
  float readFloat(Section, uint32_t);
  std::string readString(Section, uint32_t);
  bool readBoolean(Section, uint32_t);

  bool writeInt(Section, uint32_t, int64_t);
  bool writeUint(Section, uint32_t, uint64_t);
  bool writeFloat(Section, uint32_t, float);
  bool writeString(Section, uint32_t, std::string);
  bool writeBoolean(Section, uint32_t, bool);

  // Interface for Memory::Config
  Memory::Config::SRAMStructure *getSRAM();
  Memory::Config::DRAMStructure *getDRAM();
  Memory::Config::DRAMTiming *getDRAMTiming();
  Memory::Config::DRAMPower *getDRAMPower();

  // Interface for HIL::Config
  std::vector<HIL::Config::Disk> &getDiskList();
  std::vector<HIL::Config::Namespace> &getNamespaceList();
};

}  // namespace SimpleSSD

#endif
