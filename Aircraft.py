from __future__ import division
from parapy.core import *
from parapy.exchange import STEPWriter
from parapy.geom import *
from parapy.lib.xfoil import *
from Wing import Wing
from Fuselage import Fuselage
from Engine import Engine
from Landinggear import Landinggear
import math
import numpy
import mailbox


# Define class
class Aircraft(GeomBase):
    MTOW = Input(73500.0 * 9.81)
    # Values for fuselage:
    # -----------------------------
    # Cruise speed of A/C [Mach]
    cruise_speed = Input(0.7)
    # Cabin diameter of A/C [Meter]
    cabin_diameter = Input(3.0)
    # Tailcone divergence angle (max. 25)[deg]
    # that is
    # angle between cigar extension and tailcone silhouette
    tailconediv = Input(20.0)
    # Tailcone up-sweep angle [deg]
    # that is
    # angle formed between ground and lower edge of tailcone
    tailconeupsweep = Input(10.0)
    # Values for wing:
    # -----------------------------
    wing_surface = Input(122)  # A320 ref, wing area [m^2]
    c_root = Input(6.0)
    wing_config = Input("low-wing")
    wing_file_for_root = Input("airfoils\GEO_798.dat")
    wing_file_for_xfoil = Input("airfoils\NACA_0012TE.dat")
    wing_file_for_tip = Input("airfoils\GEO_798.dat")
    airfoil_eta = Input(0.95)
    # Values for engines:
    # -----------------------------
    engine_quantify = Input(2)
    engine_installation = Input("wing",
                                validator=val.OneOf(["wing", "fuselage"]))
    # Values for Landingear
    # -----------------------------
    # Diameter of Landing gear [Meter]
    landinggear_diameter = Input(1.0)
    landinggear_status = Input("deployed",
                               validator=val.OneOf(["retracted", "deployed"]))
    # -----------------------------
    # Values for Tail
    tail_config = Input("low-set")
    # Values for vertical Tail
    rudder_hingeline = Input(0.8)
    v_tail_file_for_root = Input("airfoils\NACA_661212.dat")
    v_tail_file_for_tip = Input("airfoils\NACA_63412.dat")

###############################################################################
######################### Import Attribute Parameters #########################

    @Attribute
    def import_engine(self):
        return Engine(thrust=self.engine_thrust)

    # Import the fuselage from Fuselage.py
    @Attribute
    def import_fuselage(self):
        return Fuselage(cruise_speed=self.cruise_speed,
                        cabin_diameter=self.cabin_diameter,
                        tailconediv=self.tailconediv,
                        tailconeupsweep=self.tailconeupsweep)

    # Import the horizontal Tail from Vtail.py
    @Attribute
    def import_htail(self):
        return Wing(cruise_speed=self.cruise_speed,
                    surface=10,
                    config = "h-tail",
                    file_for_root=self.wing_file_for_root,
                    file_for_tip=self.wing_file_for_tip)

    @Attribute
    def import_landinggear(self):
        return Landinggear(diameter=self.landinggear_diameter,
                           length=self.landinggear_length)

    # Import the vertical Tail from Vtail.py
    @Attribute
    def import_vtail(self):
        return Wing(cruise_speed=self.cruise_speed,
                    surface=21.5,
                    config=self.tail_config,
                    file_for_root=self.v_tail_file_for_root,
                    file_for_tip=self.v_tail_file_for_tip,
                    rudder_hingeline=self.rudder_hingeline)

    # Import the wing from Wing.py
    @Attribute
    def import_wing(self):
        return Wing(cruise_speed=self.cruise_speed,
                    config=self.wing_config,
                    file_for_root=self.wing_file_for_root,
                    file_for_tip=self.wing_file_for_tip,
                    surface=self.wing_surface,
                    c_root=self.c_root)

##############################################################################
################################# Attributes #################################

    # Display the fuselage
    @Part
    def fuselage(self):
        return TranslatedShape(shape_in=self.import_fuselage.fuselage,
                               displacement=Vector(0, 0, 0),
                               color="yellowgreen")

    # Define the lower end point of the fuselage
    # it is needed to check for tail strike and hence determine
    # the landinggear length
    @Attribute
    def tail_strike_point(self):
        return Point(-self.import_fuselage.tailconestructure[0].position.z,
                     self.import_fuselage.startvertexdown.y,
                     self.import_fuselage.startvertexdown.x)

    # Define the end point of the fuselage
    # it is needed to check for tail strike and hence determine
    # the landinggear length
    @Attribute
    def endpoint(self):
        return Point(-self.import_fuselage.tailconestructure[1].position.z,
                     self.import_fuselage.endvertex.y,
                     self.import_fuselage.endvertex.x - self.import_fuselage.tailconestructure[1].radius)

    # Determine vertical position of the wing
    @Attribute
    def vertical_wing_pos(self):
        # if it is a high wing configuration
        if self.wing_config == "high-wing":
            a = (self.cabin_diameter / 2.0) - self.import_wing.highest_point
        # if it is a low wing configuration
        else:
            a = -(self.cabin_diameter / 2.0) - self.import_wing.lowest_point
        return a

    # THIS NEEDS AN ITTERATIVE UPDATE
    @Attribute
    def horizontal_wing_pos(self):
        cylinder = numpy.abs(self.import_fuselage.tailconestructure[
                                 0].position.z) - self.import_fuselage.nose.length
        if self.engine_installation == "wing":
            x_coordinate = self.import_fuselage.nose.length + 0.5 * cylinder - self.import_wing.quarter_mac.x
        else:
            x_coordinate = self.import_fuselage.nose.length + 0.6 * cylinder - self.import_wing.quarter_mac.x
        return x_coordinate

    # Translate the wing to its designated position
    @Part
    def rightwing(self):
        return TranslatedShape(shape_in=self.import_wing.rightwing,
                               displacement=Vector(self.horizontal_wing_pos, 0,
                                                   self.vertical_wing_pos),
                               color="white")

    # Point of the leading edge of the tip airfoil
    @Attribute
    def LE_tip(self):
        return Point(self.import_wing.LE_tip.x + self.rightwing.displacement.x,
                     self.import_wing.LE_tip.y + self.rightwing.displacement.y,
                     self.import_wing.LE_tip.z + self.rightwing.displacement.z)

    @Attribute
    def quarter_mac(self):
        return Point(self.import_wing.quarter_mac.x + self.horizontal_wing_pos,
                     self.import_wing.quarter_mac.y,
                     self.import_wing.quarter_mac.z + self.vertical_wing_pos)

    @Attribute
    def aerodynamic_centre(self):
        return TranslatedCurve(curve_in=self.import_wing.aerodynamic_centre,
                               displacement=self.rightwing.displacement)

    @Part
    def leftwing(self):
        # Mirror wing, such that it generates the left semi wing
        return (MirroredShape(shape_in=self.rightwing,
                              reference_point=Point(0, 0, 0),
                              vector1=Vector(1, 0, 0),
                              vector2=Vector(0, 0, 1),
                              color="white")
                )

    @Attribute
    def landinggear_longitudinal_position(self):
        return self.horizontal_wing_pos + self.import_wing.mac + self.enginespacing[0] * 0.5 * math.tan(
            math.radians(self.import_wing.sweep))

    @Attribute
    def tipback_point(self):
        reference_to_end = math.tan(14 / 180 * math.pi) * (
            self.endpoint.x - self.landinggear_longitudinal_position) - self.cabin_diameter * 0.5
        reference_to_bottom = math.tan(14 / 180 * math.pi) * (
            self.tail_strike_point.x - self.landinggear_longitudinal_position) + self.cabin_diameter * 0.5
        if reference_to_end > reference_to_bottom:
            z_val = reference_to_end
        else:
            z_val = reference_to_bottom
        return Point(self.landinggear_longitudinal_position, 0, -z_val)

    @Attribute
    def landinggear_length(self):
        length = numpy.abs(-self.tipback_point.z + (
            self.import_wing.quarter_mac.z + self.vertical_wing_pos - self.landinggear_diameter / 2))
        if length < self.import_engine.diameter:
            length = self.import_engine.diameter
        return length

    @Part
    def leftlandinggear(self):
        return TranslatedShape(shape_in=self.import_landinggear.landinggear,
                               displacement=Vector(
                                   self.landinggear_longitudinal_position,
                                   -self.enginespacing[0] / 2,
                                   self.tipback_point.z + 0.5 * self.landinggear_diameter),
                               color="black",
                               hidden=True if self.landinggear_status == "retracted" else False)

    @Attribute
    def engine_thrust(self):
        return self.MTOW * 0.33 / self.engine_quantify

    # The engine spacing for one or two engines PER WING
    @Attribute
    def enginespacing(self):
        if self.engine_quantify == 2:
            a = 0.35 * 0.5 * self.import_wing.span
            b = 0
        else:
            a = 0.4 * 0.5 * self.import_wing.span
            b = 0.7 * 0.5 * self.import_wing.span
        return (a, b)

    @Part
    def rightengine(self):
        return TranslatedShape(quantify=int(self.engine_quantify / 2),
                               shape_in=self.import_engine.engine,
                               displacement=Vector(
                                   self.horizontal_wing_pos + math.tan(
                                       math.radians(
                                           self.import_wing.sweep)) *
                                   self.enginespacing[
                                       child.index] - self.import_engine.length,
                                   self.enginespacing[child.index],
                                   self.enginespacing[child.index] * math.tan(
                                       math.radians(
                                           self.import_wing.dihedral)) + self.vertical_wing_pos + self.import_wing.lowest_point - self.import_wing.c_root * 0.07 * 0.03 * math.cos(
                                       0.45)) if self.engine_installation == "wing" else
                               Vector(
                                   -self.import_fuselage.tailconestructure[
                                       0].position.z - 0.5 * self.import_engine.length,
                                   self.cabin_diameter / 2 + self.import_engine.diameter * (
                                       child.index + 0.75),
                                   self.vertical_wing_pos + self.import_wing.highest_point + self.import_engine.diameter * 0.5 + (
                                   (self.cabin_diameter * 0.5 + self.import_engine.diameter * 0.75) * math.tan(
                                       math.radians(self.import_wing.dihedral)))),
                               color="white")

    @Part
    def leftengine(self):
        return MirroredShape(quantify=int(self.engine_quantify / 2),
                             shape_in=self.rightengine[child.index],
                             reference_point=Point(0, 0, 0),
                             vector1=Vector(1, 0, 0),
                             vector2=Vector(0, 0, 1),
                             color="white")

    @Part
    def rightlandinggear(self):
        return MirroredShape(shape_in=self.leftlandinggear,
                             reference_point=Point(0, 0, 0),
                             vector1=Vector(1, 0, 0),
                             vector2=Vector(0, 0, 1),
                             color="black")

    @Attribute
    def V_h(self):
        return 0.083

    @Attribute
    def vertpos_vtail(self):
        return (
        self.cabin_diameter * 0.95 * 0.5)  # place the vertical tail slighly within the body such that the root of vtail is within the fuselage

    @Attribute
    def longpos_vtail_ref(self):
        return (self.import_fuselage.length * 0.95 - self.import_vtail.c_root)

    @Part
    def rudder_hinge(self):
        return TranslatedCurve(curve_in=self.import_vtail.rudder_hinge,
                               displacement=Vector(self.longpos_vtail_ref, 0, self.vertpos_vtail),
                               color="red")

    @Part
    def rudder_surface(self):
        return TranslatedSurface(surface_in=self.import_vtail.rudder_surface,
                                 displacement=Vector(self.longpos_vtail_ref, 0, self.vertpos_vtail),
                                 color="red")

    @Part
    def vtail(self):
        return TranslatedShape(shape_in=self.import_vtail.solid,
                               displacement=Vector(self.longpos_vtail_ref, 0, self.vertpos_vtail),
                               color="white",
                               transparency=0.8)

    @Part
    def htail_right(self):
        return TranslatedShape(shape_in=self.import_htail.rightwing,
                               displacement=Vector(self.import_fuselage.length * 0.75, 0, self.cabin_diameter * 0.2),
                               color="white")

    @Part
    def htail_left(self):
        # Mirror htail, such that it generates the left semi htail
        return (MirroredShape(shape_in=self.htail_right,
                              reference_point=Point(0, 0, 0),
                              vector1=Vector(1, 0, 0),
                              vector2=Vector(0, 0, 1),
                              color="white"))

    @Attribute
    def liftcurveslope(self):
        if self.wing_config == "high-wing" or self.wing_config == "low-wing":
            a = (2.0 * math.pi * self.import_wing.aspect_ratio) / (2.0 + (4 + (
            1 + (math.tan(self.import_wing.sweep_half * math.pi / 180) ** 2) / (
            self.import_wing.compressibility ** 2)) * (
                self.import_wing.aspect_ratio * self.import_wing.compressibility / self.airfoil_eta) ** 2) ** 0.5)
        else:
            a = (2.0 * math.pi * self.import_htail.aspect_ratio) / (2.0 + (
                4 + (1 + (math.tan(self.import_htail.sweep_half * math.pi / 180) ** 2) / (
                self.import_htail.compressibility ** 2)) * (
                    self.import_htail.aspect_ratio * self.import_htail.compressibility / self.airfoil_eta) ** 2) ** 0.5)
        return a


    @Attribute
    def liftcurveslope_wf(self):
        snet= self.wing_surface - (self.c_root * (self.cabin_diameter / 2.0) - (((self.cabin_diameter ** 2.0) / 8.0) * math.tan(self.import_wing.sweep * math.pi / 180.0)) + (((self.cabin_diameter ** 2.0) / 8.0) * math.tan(self.import_wing.sweep_TE * math.pi / 180.0)))
        lcs_wf = self.liftcurveslope * (1.0 + 2.15 * (self.cabin_diameter / self.import_wing.span) * (snet / self.wing_surface) + (math.pi / 2.0) * ((self.cabin_diameter ** 2.0) / self.wing_surface))
        return lcs_wf

    @Attribute
    def r(self):
        return ((self.import_htail.quarter_mac.x + self.htail_right.displacement.x) - (self.import_wing.quarter_mac.x + self.horizontal_wing_pos))/(self.import_wing.span/2.0)

    @Attribute
    def m_tv(self):
        alfa_zl = -self.alfa_zl*(math.pi/180.0)
        lh =  (0.25*self.import_htail.c_root + self.htail_right.displacement.x)-(0.25*self.c_root + self.horizontal_wing_pos)
        zdif = self.htail_right.displacement.z-self.rightwing.displacement.z
        theta = math.atan(zdif/lh)
        phi = theta + alfa_zl
        dist = ((lh**2.0)+(zdif**2.0))**0.5
        return (math.sin(phi)*dist)/(self.import_wing.span/2.0)


    @Attribute
    def downwash(self):
        r = self.r
        m = self.m_tv
        sweep = self.import_wing.sweep*(math.pi/180)
        K1 = ((0.1124 + 0.1265 * sweep + 0.1766*sweep**2.0)/r**2.0) + (0.1024/r**2.0) + 2.0
        K2 = (0.1124/r**2.0) + (0.1024/r) + 2.0
        a = (K1/K2)*(self.liftcurveslope/(math.pi*self.import_wing.aspect_ratio))*((r/(r**2.0+m**2.0))*(0.4876/(r**2+0.6319+m**2)**0.5) + (1+(r**2/(r**2 + 0.7915 + 5.0734*m**2))**0.3113)*(1-((m**2)/(1+m**2))**0.5))
        return a


    @Attribute
    def alfa_zl(self):
        xf = run_xfoil(airfoil="NACA 2412", alpha=[-2.0, 5.0, 1.0], re = 500000, pane=True)
        xf = numpy.asarray(xf)
        m = (xf[-1,1]-xf[0,1])/(xf[-1,0]-xf[0,0])
        return (-xf[2,1])/m


    @Attribute
    def xac_wf(self):
        xac_w = 0.25
        l_fn = (self.cabin_diameter/2.0)*math.tan(self.import_wing.sweep_LE*(math.pi/180.0)) + self.rightwing.displacement.x
        c_g = self.import_wing.surface/self.import_wing.span
        xac_f = -((1.8*l_fn*self.cabin_diameter**2.0)/(self.liftcurveslope_wf*self.import_wing.surface*self.import_wing.mac)) + ((0.273/(1+self.import_wing.taper_ratio))*((self.cabin_diameter*c_g*(self.import_wing.span-self.cabin_diameter)*math.tan(self.import_wing.sweep))/((self.import_wing.span+2.15*self.cabin_diameter)*self.import_wing.mac**2)))
        return xac_w + xac_f

    @Attribute
    def xac_nacelles(self):
        b_n = self.import_engine.diameter
        if self.engine_installation == "wing":
            k_n = -4.0
            if self.engine_quantify == 2:
                l_n = self.import_wing.quarter_mac.x-self.rightengine[0].displacement.x
                xac_n = 2.0*(k_n * ((l_n*b_n**2.0)/(self.import_wing.surface*self.import_wing.mac*self.liftcurveslope_wf)))
            else:
                l_n1 = self.import_wing.quarter_mac.x - self.rightengine[0].displacement.x
                l_n2 = self.import_wing.quarter_mac.x - self.rightengine[1].displacement.x
                xac_n1 = 2.0 * (k_n * ((l_n1 * b_n ** 2.0) / (self.import_wing.surface * self.import_wing.mac * self.liftcurveslope_wf)))
                xac_n2 = 2.0 * (k_n * ((l_n2 * b_n ** 2.0) / (self.import_wing.surface * self.import_wing.mac * self.liftcurveslope_wf)))
                xac_n = xac_n1 + xac_n2
        else:
            k_n = -2.5
            if self.engine_quantify == 2:
                l_n = (self.rightengine[0].displacement.x - self.import_wing.quarter_mac.x) + self.import_engine.length
                xac_n = 2.0*(k_n * ((l_n*b_n**2.0)/(self.import_wing.surface*self.import_wing.mac*self.liftcurveslope_wf)))
            else:
                l_n1 = self.rightengine[0].displacement.x - self.import_wing.quarter_mac.x + self.import_engine.length
                l_n2 = self.rightengine[1].displacement.x - self.import_wing.quarter_mac.x + self.import_engine.length
                xac_n1 = 2.0 * (k_n * ((l_n1 * b_n ** 2.0) / (self.import_wing.surface * self.import_wing.mac * self.liftcurveslope_wf)))
                xac_n2 = 2.0 * (k_n * ((l_n2 * b_n ** 2.0) / (self.import_wing.surface * self.import_wing.mac * self.liftcurveslope_wf)))
                xac_n = xac_n1 + xac_n2
        return xac_n

    @Attribute
    def xac(self):
        return self.xac_nacelles + self.xac_wf




    @Attribute
    def deep_stall_check(self):
        y = (self.htail_right.displacement.z-self.rightwing.displacement.z)/self.import_wing.mac
        x = (self.r*(self.import_wing.span/(2.0*self.import_wing.mac)))
        y1 = (-0.0368 * x ** 2.0 + 0.5359 * x + 0.0298)
        y2 = (-0.0105 * x ** 2.0 + 0.2123 * x - 0.114)
        y3 = (0.0098 * x ** 2.0 - 0.0953 * x + 0.0084)

        if y > y2:
            a = self.mbox('Design check for deep stall',
              'Design is not safe because the tail is in deep stall zone.', 0)
        return a



    @Attribute
    def mbox(self):
        return mailbox.mbox


        # # Write a stepfile of the aircraft
        # @Part
        # def stepfile(self):
        #     return STEPWriter([self.first_wing, self.clonedwing, self.fuselage],
        #                       'acstep.step')



if __name__ == '__main__':
    from parapy.gui import display

    obj = Aircraft()
    display(obj)