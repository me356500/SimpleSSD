// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Copyright (C) 2019 CAMELab
 *
 * Author: Donghyun Gouk <kukdh1@camelab.org>
 */

#pragma once

#ifndef __SIMPLESSD_MEM_DEF_HH__
#define __SIMPLESSD_MEM_DEF_HH__

#include "sim/object.hh"

namespace SimpleSSD::Memory {

class Request {
 public:
  uint64_t offset;
  uint64_t length;
  Event eid;
  uint64_t data;
  uint64_t beginAt;

  Request() : offset(0), length(0), eid(InvalidEventID) {}
  Request(uint64_t a, uint64_t l, Event e, uint64_t d)
      : offset(a), length(l), eid(e), data(d) {}
  Request(const Request &) = delete;
  Request(Request &&) noexcept = default;

  Request &operator=(const Request &) = delete;
  Request &operator=(Request &&) = default;

  static void backup(std::ostream &out, Request *item) {
    BACKUP_EVENT(out, item->eid);
    BACKUP_SCALAR(out, item->offset);
    BACKUP_SCALAR(out, item->length);
    BACKUP_SCALAR(out, item->beginAt);
  }

  static Request *restore(std::istream &in, ObjectData &object) {
    auto item = new Request();

    RESTORE_EVENT(in, item->eid);
    RESTORE_SCALAR(in, item->offset);
    RESTORE_SCALAR(in, item->length);
    RESTORE_SCALAR(in, item->beginAt);

    return item;
  }
};

}  // namespace SimpleSSD::Memory

#endif
