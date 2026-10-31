#pragma once
// =====================================================================
//  Null Island Space Program  --  which build is this?
// ---------------------------------------------------------------------
//  CMake writes app_version.h into the build directory on every build
//  (main/app_version.h.in): the git commit, the build time, and the
//  release out of metadata.json. Anything that REPORTS the build reads
//  it through here, so there is one copy of the fallback and not one
//  per reporter.
//
//  "unknown" survives only for a build with no git and no generated
//  header at all.
// =====================================================================

#if defined(__has_include)
#if __has_include("app_version.h")
#include "app_version.h"
#endif
#endif

#ifndef APP_GIT_HASH
#define APP_GIT_HASH "unknown"
#endif
#ifndef APP_BUILD_TIME
#define APP_BUILD_TIME "unknown"
#endif
#ifndef APP_VERSION
#define APP_VERSION ""
#endif
