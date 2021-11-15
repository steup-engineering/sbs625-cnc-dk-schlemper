#!/usr/bin/python
#    Copyright 2007 Chris Radek
#    Derived from a work by John Kasunich and Jeff Epler
#
#    This program is free software; you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation; either version 2 of the License, or
#    (at your option) any later version.
#
#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with this program; if not, write to the Free Software
#    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA


from vismach import *
import hal
import math
import sys

# give endpoint Z values and radii
# resulting cylinder is on the Z axis
class HalToolCylinder(CylinderY):
	def __init__(self, comp):
		CylinderY.__init__(self, 0, 0, 0, 0)
		self.comp = comp

	def coords(self):
		if self.comp.tooloffset_a == 0:
			return -(self.comp.tooloffset_v), self.comp.tooloffset_w, self.comp.tooloffset_v, self.comp.tooloffset_w
		else:
			return 0, 0, self.comp.tooloffset_v, 50

c = hal.component("bridgesawgui")
c.newpin("joint_x", hal.HAL_FLOAT, hal.HAL_IN)
c.newpin("joint_y", hal.HAL_FLOAT, hal.HAL_IN)
c.newpin("joint_z", hal.HAL_FLOAT, hal.HAL_IN)
c.newpin("joint_a", hal.HAL_FLOAT, hal.HAL_IN)
c.newpin("joint_c", hal.HAL_FLOAT, hal.HAL_IN)
c.newpin("tooloffset_a", hal.HAL_FLOAT, hal.HAL_IN)
c.newpin("tooloffset_v", hal.HAL_FLOAT, hal.HAL_IN)
c.newpin("tooloffset_w", hal.HAL_FLOAT, hal.HAL_IN)
c.ready()

apivot_toolaxis = -3.0;
apivot_bladepuck = -350.0;
cpivot_toolaxis = 2.0;
cpivot_apivot = 200.0;

for setting in sys.argv[1:]: exec setting

tooltip = Capture()

tool = Color([1, 0.5, 0, 0], [
	HalToolCylinder(c)
	])

tool = Collection([
	tool,
	HalTranslate([tooltip], c, "tooloffset_w", 0, 0, -1),
	])

tool = HalTranslate([tool], c, "tooloffset_v", 0, -1, 0)

tool = Translate([tool], 0, apivot_bladepuck, 0)

motor = Color([0,1,0,0], [
	CylinderY(apivot_bladepuck, 125, apivot_bladepuck + 50, 125),
	Box(-150, apivot_bladepuck + 50, -125, 150, 300, 125)
	])

tool = Collection([
	tool,
	motor,
	CylinderY(apivot_bladepuck - 50, 50, apivot_bladepuck, 50),
	])

tool = Translate([tool], 0, 0, apivot_toolaxis)

tool = HalRotate([tool], c, "joint_a", 1, 1, 0, 0)

tool = Collection([
	tool,
	CylinderX(-195 , 50, 195, 50),
	])

tool = Translate([tool], 0, cpivot_apivot, 0)

holder = Color([1,1,0,0], [
	Box(-160, -50, -120, -180, cpivot_apivot + 100, 150),
	Box( 160, -50, -120,  180, cpivot_apivot + 100, 150),
	Box(-180, -50,  150,  180, 50, 180)
	])

wrist = Collection([
	tool,
	holder
	])

wrist = Translate([wrist], cpivot_toolaxis, 0, 0)

wrist = HalRotate([wrist],c,"joint_c",1,0,0,1)

wrist = Collection([
	wrist,
	CylinderZ(180, 50, 350, 50)
	])

support = Color([1,1,0,0], [
	Box(-100, -100, 350, 100, 100, 1700),
	])

ram = Collection([
	wrist,
	support
	])

ram = HalTranslate([ram],c,"joint_z",0,0,1)

ram = Translate([ram], 0, -150, -600)

ram = Collection([
	ram,
	Box(-200, -100, -200, 200, 0, 200)
	])
    
ram = HalTranslate([ram],c,"joint_x",1,0,0)

ram = Translate([ram], -1890, -200, 0)

bridge = Color([1,1,0,0], [
	Box(-2750, -200, -200, 2750, 200, 200)
	])

ram = Collection([
	ram,
	bridge
	])

ram = HalTranslate([ram],c,"joint_y",0,1,0)

ram = Translate([ram], 0, 1380, 280)

ram = Collection([
	ram,
	Box(-2750, -2750, -80,  -2590,  2750,    80),
	Box(-2750, -2750, -80,  -2590, -2590, -1880),
	Box(-2750,   -80, -80,  -2590,    80, -1880),
	Box(-2750,  2750, -80,  -2590,  2590, -1880),
	Box( 2750, -2750, -80,   2590, 2750,    80),
	Box( 2750, -2750, -80,   2590, -2590, -1880),
	Box( 2750,   -80, -80,   2590,    80, -1880),
	Box( 2750,  2750, -80,   2590,  2590, -1880)
	])

work = Capture()

table = Color([0.5,0.5,0.5,0], [
	Box(-2500, -2500, -1300, 2500, 2500, -1350)
        ])

table = Collection([
	work,
	table
        ])

model = Collection([
	ram,
	table
	])

main(model, tooltip, work, 1500)

