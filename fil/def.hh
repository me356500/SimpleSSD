// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Copyright (C) 2019 CAMELab
 *
 * Author: Donghyun Gouk <kukdh1@camelab.org>
 */

#pragma once

#ifndef __SIMPLESSD_FIL_DEF_HH__
#define __SIMPLESSD_FIL_DEF_HH__

#include <cinttypes>
#include <functional>

#include "sim/event.hh"

namespace SimpleSSD::FIL {

enum PageAllocation : uint8_t {
  None = 0,
  Channel = 1,
  Way = 2,
  Die = 4,
  Plane = 8,
  All = 15,
};

enum Index : uint8_t {
  Level1,
  Level2,
  Level3,
  Level4,
};

enum class Operation : uint8_t {
  None,
  Read,
  ReadCache,
  ReadCopyback,
  Program,
  ProgramCache,
  ProgramCopyback,
  Erase,
};

class Request {
 private:
  bool multiplane;
  Operation opcode;

  PPN address;

  uint8_t *buffer;

  Event eid;
  uint64_t data;

 public:
  Request(PPN a, Event e, uint64_t d)
      : multiplane(false),
        opcode(Operation::None),
        address(a),
        pageData(nullptr),
        pageSpare(nullptr),
        eid(e),
        data(d) {}

  void setData(uint8_t *ptr) { buffer = ptr; }

  const uint8_t *getData() { return buffer; }
};

}  // namespace SimpleSSD::FIL

#endif
