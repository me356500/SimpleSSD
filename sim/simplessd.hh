// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Copyright (C) 2019 CAMELab
 *
 * Author: Donghyun Gouk <kukdh1@camelab.org>
 */

#pragma once

#ifndef __SIM_SIMPLESSD_HH__
#define __SIM_SIMPLESSD_HH__

#include "sim/config.hh"
#include "sim/engine.hh"
#include "sim/interface.hh"
#include "sim/log.hh"

namespace SimpleSSD {

/**
 * \brief SimpleSSD object declaration
 *
 * SimpleSSD object. It contains everything about a SSD.
 * You can create multiple SimpleSSD objects to create multiple SSD.
 * You cannot copy this object.
 */
class SimpleSSD {
 private:
  bool inited;  //!< Flag whether this object is initialized

  Config *config;        //!< Config object provided by simulation system
  Engine *engine;        //!< Engine object provided by simulation system
  Log log;               //!< Log system
  Interface *interface;  //!< Interface object provided by simulation system

  std::ostream *outfile;
  std::ostream *errfile;
  std::ostream *debugfile;

  void joinPath(std::string &, std::string &) noexcept;
  void openStream(std::ostream *, std::string &, std::string &) noexcept;

 public:
  SimpleSSD();
  SimpleSSD(const SimpleSSD &) = delete;
  SimpleSSD(SimpleSSD &&) = default;
  ~SimpleSSD();

  SimpleSSD &operator=(const SimpleSSD &) = delete;
  SimpleSSD &operator=(SimpleSSD &&) noexcept = default;

  bool init(Config *, Engine *, Interface *) noexcept;
  void deinit() noexcept;

  void read(uint64_t, uint64_t, uint8_t *, Event, void * = nullptr) noexcept;
  uint64_t read(uint64_t, uint64_t, uint8_t *) noexcept;
  void write(uint64_t, uint64_t, uint8_t *, Event, void * = nullptr) noexcept;
  uint64_t write(uint64_t, uint64_t, uint8_t *) noexcept;

  void createCheckpoint() noexcept;
  void restoreCheckpoint() noexcept;
};

}  // namespace SimpleSSD

#endif
