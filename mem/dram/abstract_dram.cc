// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Copyright (C) 2019 CAMELab
 *
 * Author: Donghyun Gouk <kukdh1@camelab.org>
 */

#include "mem/dram/abstract_dram.hh"

#include <cstring>

#include "util/algorithm.hh"

namespace SimpleSSD::Memory::DRAM {

AbstractDRAM::AbstractDRAM(ObjectData &o) : AbstractRAM(o) {
  pStructure = o.config->getDRAM();
  pTiming = o.config->getDRAMTiming();
  pPower = o.config->getDRAMPower();
}

AbstractDRAM::~AbstractDRAM() {}

void AbstractDRAM::getStatList(std::vector<Stat> &list,
                               std::string prefix) noexcept {
  list.emplace_back(prefix + "read.request_count", "Read request count");
  list.emplace_back(prefix + "read.bytes", "Read data size in byte");
  list.emplace_back(prefix + "write.request_count", "Write request count");
  list.emplace_back(prefix + "write.bytes", "Write data size in byte");
  list.emplace_back(prefix + "request_count", "Total request count");
  list.emplace_back(prefix + "bytes", "Total data size in byte");
}

void AbstractDRAM::getStatValues(std::vector<double> &values) noexcept {
  values.push_back((double)readStat.count);
  values.push_back((double)readStat.size);
  values.push_back((double)writeStat.count);
  values.push_back((double)writeStat.size);
  values.push_back((double)(readStat.count + writeStat.count));
  values.push_back((double)(readStat.size + writeStat.size));
}

void AbstractDRAM::resetStatValues() noexcept {
  readStat.clear();
  writeStat.clear();
}

void AbstractDRAM::createCheckpoint(std::ostream &out) const noexcept {
  BACKUP_SCALAR(out, readStat);
  BACKUP_SCALAR(out, writeStat);
}

void AbstractDRAM::restoreCheckpoint(std::istream &in) noexcept {
  RESTORE_SCALAR(in, readStat);
  RESTORE_SCALAR(in, writeStat);
}

}  // namespace SimpleSSD::Memory::DRAM
