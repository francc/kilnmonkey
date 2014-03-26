//  Kiln Monkey - Debug Resource - Copyright 2012 by Francisco Castro <http://fran.cc>
//
//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program.  If not, see <http://www.gnu.org/licenses/>.

class Debug : public Resource {
    uint8_t get(Request &request);
};

uint8_t Debug::get(Request &request)
{
    request.writer.openMap();
    request.writer.putString("memory");
    request.writer.putInteger(freeMemory());
    request.writer.putString("loop");
    request.writer.putInteger(loop_time);
    request.writer.close();
    return TPM_205_Content;
}
 
