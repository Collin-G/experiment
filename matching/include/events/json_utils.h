#pragma once
#include "events/events.h"


EngineCommand parse(const std::string& raw);
std::string serialize(const EngineEvent& ev);
