/*
 * Copyright (c) 2019 Pilz GmbH & Co. KG
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.

 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef REGISTER_CONTAINER_H
#define REGISTER_CONTAINER_H

#include <vector>
#include <cstdint>

namespace prbt_hardware_support
{
//! Convenience data type defining the data type for a collection of registers.
using RegCont = std::vector<uint16_t>;

}  // namespace prbt_hardware_support

#endif  // REGISTER_CONTAINER_H
