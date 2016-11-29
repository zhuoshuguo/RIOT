/*
 * Copyright (C) 2015 Daniel Krebs
 *               2016 INRIA
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @{
 * @ingroup     net_gnrc_mac
 * @file
 * @brief       Internal functions of GNRC_WMAC
 *
 * @author      Daniel Krebs <github@daniel-krebs.net>
 * @author      Shuguo Zhuo  <shuguo.zhuo@inria.fr>
 * @}
 */

#include <stdbool.h>
#include <net/gnrc.h>
#include <net/gnrc/mac/internal.h>

#define ENABLE_DEBUG    (0)
#include "debug.h"
