// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Copyright (C) 2019 CAMELab
 *
 * Author: Donghyun Gouk <kukdh1@camelab.org>
 */

#include "hil/command_manager.hh"

namespace SimpleSSD::HIL {

CommandManager::CommandManager(ObjectData &o) : Object(o) {}

CommandManager::~CommandManager() {}

SubCommand &CommandManager::getSubCommand(uint64_t tag, Status status) {
  auto iter = commandList.find(tag);

  panic_if(iter == commandList.end(), "No such command exists.");

  auto &cmd = iter->second;
  auto scmd = cmd.subCommandList.begin();

  for (; scmd != cmd.subCommandList.end(); ++scmd) {
    if (scmd->status == status) {
      break;
    }
  }

  return *scmd;
}

SubCommand &CommandManager::getSubCommand(uint64_t tag, uint32_t id) {
  auto iter = commandList.find(tag);

  panic_if(iter == commandList.end(), "No such command exists.");

  return iter->second.subCommandList.at(id);
}

void CommandManager::createCommand(uint64_t tag, Event eid) {
  auto iter = commandList.emplace(std::make_pair(tag, Command(eid)));

  panic_if(!iter.second, "Command with tag %" PRIu64 " already exists.", tag);
}

uint32_t CommandManager::createSubCommand(uint64_t tag) {
  auto iter = commandList.find(tag);

  panic_if(iter == commandList.end(), "No such command exists.");

  uint32_t ret = iter->second.subCommandList.size();

  iter->second.subCommandList.emplace_back(SubCommand());

  return ret;
}

void CommandManager::destroyCommand(uint64_t tag) {
  auto iter = commandList.find(tag);

  panic_if(iter == commandList.end(), "No such command exists.");

  commandList.erase(iter);
}

void CommandManager::getStatList(std::vector<Stat> &, std::string) noexcept {}

void CommandManager::getStatValues(std::vector<double> &) noexcept {}

void CommandManager::resetStatValues() noexcept {}

void CommandManager::createCheckpoint(std::ostream &out) const noexcept {
  uint64_t size = commandList.size();

  BACKUP_SCALAR(out, size);

  for (auto &iter : commandList) {
    BACKUP_SCALAR(out, iter.first);

    BACKUP_EVENT(out, iter.second.eid);

    size = iter.second.subCommandList.size();
    BACKUP_SCALAR(out, size);

    for (auto &scmd : iter.second.subCommandList) {
      BACKUP_SCALAR(out, scmd.status);
      BACKUP_SCALAR(out, scmd.opcode);
      BACKUP_SCALAR(out, scmd.lpn);
      BACKUP_SCALAR(out, scmd.ppn);

      size = scmd.buffer.size();
      BACKUP_SCALAR(out, size);

      if (size > 0) {
        BACKUP_BLOB(out, scmd.buffer.data(), size);
      }

      size = scmd.spare.size();
      BACKUP_SCALAR(out, size);

      if (size > 0) {
        BACKUP_BLOB(out, scmd.spare.data(), size);
      }
    }
  }
}

void CommandManager::restoreCheckpoint(std::istream &in) noexcept {
  uint64_t size, size2, size3;

  RESTORE_SCALAR(in, size);

  for (uint64_t i = 0; i < size; i++) {
    uint64_t tag;
    Event eid;

    RESTORE_SCALAR(in, tag);
    RESTORE_EVENT(in, eid);

    Command cmd(eid);

    RESTORE_SCALAR(in, size2);

    for (uint64_t j = 0; j < size2; j++) {
      SubCommand scmd;

      RESTORE_SCALAR(in, scmd.status);
      RESTORE_SCALAR(in, scmd.opcode);
      RESTORE_SCALAR(in, scmd.lpn);
      RESTORE_SCALAR(in, scmd.ppn);
      RESTORE_SCALAR(in, size3);

      if (size3 > 0) {
        scmd.buffer.resize(size3);

        RESTORE_BLOB(in, scmd.buffer.data(), size);
      }

      RESTORE_SCALAR(in, size3);

      if (size3 > 0) {
        scmd.spare.resize(size3);

        RESTORE_BLOB(in, scmd.spare.data(), size);
      }

      cmd.subCommandList.emplace_back(scmd);
    }

    commandList.emplace(std::make_pair(tag, cmd));
  }
}

}  // namespace SimpleSSD::HIL
