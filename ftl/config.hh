// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Copyright (C) 2019 CAMELab
 *
 * Author: Donghyun Gouk <kukdh1@camelab.org>
 */

#pragma once

#ifndef __SIMPLESSD_FTL_CONFIG_HH__
#define __SIMPLESSD_FTL_CONFIG_HH__

#include <vector>

#include "fil/def.hh"
#include "sim/base_config.hh"

namespace SimpleSSD::FTL {

class Config : public BaseConfig {
 public:
  enum Key : uint32_t {
    MappingMode,

    // Filling
    FillingMode,
    FillRatio,
    InvalidFillRatio,

    // Garbage Collection
    VictimSelectionPolicy,
    DChoiceParam,
    GCThreshold,

    // Common FTL setting
    OverProvisioningRatio,
    SuperpageAllocation,
    MergeReadModifyWrite,
    AllowPageLevelRead,

    // VLFTL
    VLTableRatio,
    MergeThreshold,
  };

  enum class MappingType : uint8_t {
    PageLevelFTL,
    PageLevelWithDemandPaging,
    BlockLevelFTL,
    VLFTL,
  };

  enum class FillingType : uint8_t {
    SequentialSequential,
    SequentialRandom,
    RandomRandom,
  };

  enum class VictimSelectionMode : uint8_t {
    Greedy,
    Random,
    CostBenefit,
    DChoice,
  };

  struct PageTableStructure {
    uint32_t levels;
    std::vector<LPN> masks;
  };

 private:
  MappingType mappingMode;
  float overProvision;
  FillingType fillingMode;
  float fillRatio;
  float invalidFillRatio;

  bool mergeRMW;
  bool allowPageLevelRead;

  VictimSelectionMode gcBlockSelection;
  uint64_t dChoiceParam;
  float gcThreshold;
  uint8_t superpageAllocation;
  float pmTableRatio;
  float mergeThreshold;

  PageTableStructure pageTable;

  std::string superpage;

 public:
  Config();

  const char *getSectionName() override { return "ftl"; }

  void loadFrom(pugi::xml_node &) override;
  void storeTo(pugi::xml_node &) override;
  void update() override;

  uint64_t readUint(uint32_t) override;
  float readFloat(uint32_t) override;
  bool readBoolean(uint32_t) override;
  bool writeUint(uint32_t, uint64_t) override;
  bool writeFloat(uint32_t, float) override;
  bool writeBoolean(uint32_t, bool) override;

  PageTableStructure *getPageTable();
};

}  // namespace SimpleSSD::FTL

#endif
