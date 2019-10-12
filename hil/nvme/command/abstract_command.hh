// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Copyright (C) 2019 CAMELab
 *
 * Author: Donghyun Gouk <kukdh1@camelab.org>
 */

#pragma once

#ifndef __SIMPLESSD_HIL_NVME_COMMAND_ABSTRACT_COMMAND_HH__
#define __SIMPLESSD_HIL_NVME_COMMAND_ABSTRACT_COMMAND_HH__

#include <queue>

#include "hil/common/interrupt_manager.hh"
#include "hil/common/dma_engine.hh"
#include "sim/interface.hh"
#include "sim/object.hh"

namespace SimpleSSD::HIL::NVMe {

#define debugprint_command(format, ...)                                        \
  {                                                                            \
    uint64_t _uid = getUniqueID();                                             \
                                                                               \
    debugprint(Log::DebugID::HIL_NVMe_Command,                                 \
               "CTRL %-3u | SQ %2u:%-5u | " format, (uint16_t)(_uid >> 32),    \
               (uint16_t)(_uid >> 16), (uint16_t)_uid, ##__VA_ARGS__);         \
  }

class Subsystem;
class Controller;
class Arbitrator;
class SQContext;
class CQContext;
class ControllerData;

class CommandData {
 public:
  Subsystem *subsystem;
  Controller *controller;
  Interface *interface;
  Arbitrator *arbitrator;
  InterruptManager *interrupt;
  DMAEngine *dmaEngine;

  CommandData(Subsystem *, ControllerData *);
};

/**
 * \brief Command object
 *
 * All NVMe command inherits this object.
 */
class Command : public Object {
 protected:
  CommandData data;

  DMATag dmaTag;

  SQContext *sqc;
  CQContext *cqc;

  void createResponse();
  void createDMAEngine(uint32_t, Event);

 public:
  Command(ObjectData &, Subsystem *, ControllerData *);
  Command(const Command &) = delete;
  Command(Command &&) noexcept = default;
  virtual ~Command();

  CQContext *getResult();
  CommandData &getCommandData();
  uint64_t getUniqueID();
  virtual void setRequest(SQContext *) = 0;

  void getStatList(std::vector<Stat> &, std::string) noexcept override;
  void getStatValues(std::vector<double> &) noexcept override;
  void resetStatValues() noexcept override;

  void createCheckpoint(std::ostream &) const noexcept override;
  void restoreCheckpoint(std::istream &) noexcept override;
};

}  // namespace SimpleSSD::HIL::NVMe

#endif
