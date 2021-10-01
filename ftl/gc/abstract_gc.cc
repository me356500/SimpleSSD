// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Copyright (C) 2019 CAMELab
 *
 * Author: Donghyun Gouk <kukdh1@camelab.org>
 */

#include "ftl/gc/abstract_gc.hh"

#include "ftl/mapping/abstract_mapping.hh"

namespace SimpleSSD::FTL::GC {

AbstractGC::AbstractGC(ObjectData &o, FTLObjectData &fo, FIL::FIL *fil)
    : AbstractJob(o, fo), pFIL(fil), param(nullptr) {}

AbstractGC::~AbstractGC() {}

void AbstractGC::initialize() {
  param = ftlobject.pMapping->getInfo();
}

}  // namespace SimpleSSD::FTL::GC
