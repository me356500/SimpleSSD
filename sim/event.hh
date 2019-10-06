// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Copyright (C) 2019 CAMELab
 *
 * Author: Donghyun Gouk <kukdh1@camelab.org>
 */

#pragma once

#ifndef __SIM_EVENT_HH__
#define __SIM_EVENT_HH__

#include <cinttypes>
#include <functional>

#include "sim/checkpoint.hh"

namespace SimpleSSD {

/**
 * \brief Event ID definition
 *
 * Unique ID of events in SimpleSSD. Event ID 0 is invalid.
 */
using Event = uint64_t;

const Event InvalidEventID = 0;

/**
 * \brief Event Context object
 *
 * This object stores user data (and its size) when checkpointing. Only with
 * pointer (void *), we cannot store struct to binary file -- we don't know the
 * original size!.
 *
 * If you store class with virtual functions, first 8 bytes of the class data
 * is pointer to vtable. If you compile SimpleSSD with different compiler or
 * option, vtable posiion may be changed - restored class may have wrong vtable
 * pointer.
 *
 * Sometimes, you want to store EventContext in EventContext. To prevent double
 * memory allocation, I prevent to store value (it needs copy with memory
 * allocation). Just use get/setSubdata for such purpose.
 */
class EventContext {
 private:
  uint64_t size;
  void *data;

  uint64_t subsize;
  void *subdata;

  EventContext(uint64_t s, void *p) : size(s), data(p) {}

 public:
  EventContext() : size(0), data(nullptr) {}

  template <class Type,
            std::enable_if_t<!std::is_pointer_v<Type>, Type> * = nullptr>
  EventContext(Type t) noexcept = delete;

  template <class Type,
            std::enable_if_t<std::is_pointer_v<Type>, Type> * = nullptr>
  EventContext(Type t) noexcept : size(sizeof(*t)), data(t) {}

  template <class Type,
            std::enable_if_t<!std::is_pointer_v<Type>, Type> * = nullptr>
  Type get() noexcept = delete;

  template <class Type,
            std::enable_if_t<std::is_pointer_v<Type>, Type> * = nullptr>
  Type get() noexcept {
    return (Type)data;
  }

  void setSubdata(EventContext &sub) noexcept {
    subsize = sub.size;
    subdata = sub.data;
  }

  EventContext getSubdata() noexcept {
    return EventContext(subsize, subdata);
  }

  void backup(std::ostream &out) noexcept {
    BACKUP_SCALAR(out, size);

    if (size > 0) {
      BACKUP_BLOB(out, data, size);
    }

    BACKUP_SCALAR(out, subsize);

    if (subsize > 0) {
      BACKUP_BLOB(out, subdata, subsize);
    }
  }

  void restore(std::istream &in) noexcept {
    RESTORE_SCALAR(in, size);

    if (size > 0) {
      data = malloc(size);

      RESTORE_BLOB(in, data, size);
    }

    RESTORE_SCALAR(in, subsize);

    if (subsize > 0) {
      subdata = malloc(subsize);

      RESTORE_BLOB(in, subdata, size);
    }
  }
};

/**
 * \brief Event function definition
 *
 * Event function will be called when the event triggered.
 * First param is current tick, second param is user data passed in schedule
 * function.
 */
using EventFunction = std::function<void(uint64_t, EventContext)>;

class Request {
 public:
  uint64_t offset;
  uint64_t length;
  Event eid;
  EventContext context;
  uint64_t beginAt;

  Request() : offset(0), length(0), eid(InvalidEventID) {}
  Request(uint64_t a, uint64_t l, Event e, EventContext c)
      : offset(a), length(l), eid(e), context(c) {}
  Request(const Request &) = delete;
  Request(Request &&) noexcept = default;

  Request &operator=(const Request &) = delete;
  Request &operator=(Request &&) = default;
};

class RequestWithData : public Request {
 public:
  uint8_t *buffer;

  RequestWithData() : Request(), buffer(nullptr) {}
  RequestWithData(uint64_t a, uint64_t l, Event e, EventContext c, uint8_t *b)
      : Request(a, l, e, c), buffer(b) {}
  RequestWithData(const RequestWithData &) = delete;
  RequestWithData(RequestWithData &&) noexcept = default;

  RequestWithData &operator=(const RequestWithData &) = delete;
  RequestWithData &operator=(RequestWithData &&) = default;
};

}  // namespace SimpleSSD

#endif
