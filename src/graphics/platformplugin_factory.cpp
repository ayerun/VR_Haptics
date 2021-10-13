// Copyright (c) 2017-2021, The Khronos Group Inc.
//
// SPDX-License-Identifier: Apache-2.0

#include "pch.h"
#include "platformplugin.h"

std::shared_ptr<IPlatformPlugin> CreatePlatformPlugin_Xlib(const std::shared_ptr<Options>&);

std::shared_ptr<IPlatformPlugin> CreatePlatformPlugin(const std::shared_ptr<Options>& options) {
    return CreatePlatformPlugin_Xlib(options);
}
