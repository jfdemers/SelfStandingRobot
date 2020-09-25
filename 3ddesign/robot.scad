// Create a tire with the specified radius r, height h and thread height th.
module tire(r=45, thickness=2, h=8.5, th=2) {
    perimeter = 2 * PI * r;
    thread_spacing = 3 * 360 / perimeter;
    
    rotate([0,90,0])
        union() {
            difference() {
                cylinder(r=r+thickness, h=h);
                translate([0, 0, -0.5]) cylinder(r=r, h=h+1);
            }
            
            for (i = [0:thread_spacing:360 - thread_spacing]) {
                    rotate([0, 0, i])
                        translate([r+thickness-0.5, 0, 0])
                            cube([th, 1.0, h]);
            }
        }
}


module wheel_hub(wheel=10, hub=5, shaft=2.5, wall=3) {
    translate([0, 0, -hub])
        rotate_extrude() {
            translate([shaft, 0, 0])
                square([wall, wheel+hub]);
        }
}

module wheel_contour(thickness=10, wall=3, radius=45, lip=2, lip_height=2) {
    rotate_extrude() {
        polygon([[radius-wall, 0], [radius+lip_height, 0], [radius+lip_height, lip], [radius, lip], [radius, lip+thickness], [radius+lip_height, lip+thickness], [radius+lip_height, 2*lip+thickness], [radius-wall, 2*lip+thickness]]);
    }
}

module wheel_spoke(thickness=10, wall=3, radius=45, hub=5, angle=30, offset=5) {
    rotate_extrude(angle=angle) {
        translate([hub - 0.1, 0])
            square([radius - hub - wall + 0.2, thickness - offset]);
    }
}

module wheel_spokes(thickness=10, wall=3, radius=45, hub=5, count=6, offset=5) {
    angle = 360 / count / 2;

    union() {
        for (i = [0:count]) {
            rotate([0, 0, 2 * i * angle])
                wheel_spoke(thickness, wall, radius, hub, angle, offset);
        }
    }
}

module wheel(radius = 45, thickness=13, wall = 3, shaft = 2.5, hub_wall = 3, hub_extra = 2, spokes = 6, lip = 2, lip_height = 2) {
    rotate([0, 90, 0])
        union() {
            wheel_hub(wheel=thickness, hub=hub_extra, shaft=shaft, wall=hub_wall);
            wheel_contour(thickness=thickness - 2 * lip, wall=wall, radius = radius, lip = lip, lip_height = lip_height);
            wheel_spokes(thickness=thickness, wall=wall, radius=radius, hub=shaft+hub_wall, count=spokes, offset=thickness / 2 + 1);
        }
}

module wheel_tire() {
    color("gray") wheel();
    color("orange")
        translate([2.25,0,0]) tire();
}

module nut_holder() {
    cube([7, 10, 10]);
}

module circumscribed_polygon(r=1, f=6) {
    a = [for (i = [0:f]) [cos(i * 360 / f) * r, sin(i * 360 / f) * r]];
    polygon(a);
} 

module m3_nut(margin = 0) {
    linear_extrude(2.4) circumscribed_polygon(r=(6.35 + margin) / 2, f=6);
}

module m3_screw_in_hole(depth = 8) {
    difference() {
        cylinder(r=2.5, h=depth);
        translate([0, 0, -0.1]) cylinder(r=1.25, h=depth + 0.2);
    }
}

module right_plate() {
    cube([60, 150, 4]);
    translate([53, 0, 4]) nut_holder();
    translate([53, 140, 4]) nut_holder();
}

module breadboard_row(pins) {
    color("gold") {
        for (i = [0:pins - 1]) {
            translate([2.54 * i, 0, 0]) {
                cylinder(r=0.9, h=1.61);
            }
        }
    }
}

module medium_breadboard() {
    center = 63 / 2;
    x_offset = (94 - 29 * 2.54) / 2;
    y_offset = 2.54 + 2.54 / 2;

    hole_offset = 4;

    difference() {
        union() {
            color("red") cube([94, 63, 1.6]);
            for (y = [0:4]) {
                translate([x_offset, center + y_offset + y * 2.54, 0]) breadboard_row(30);
            }

            for (y = [0:4]) {
                translate([x_offset, center - y_offset - y * 2.54, -0.005]) breadboard_row(30);
            }

            for (y = [0:1]) {
                translate([x_offset, center + y_offset + (7 + y) * 2.54, -0.005]) breadboard_row(30);
                translate([x_offset, center - y_offset - (7 + y) * 2.54, -0.005]) breadboard_row(30);
            }
        }

        translate([hole_offset, hole_offset, -0.05]) cylinder(r=3.7/2, h=1.7);
        translate([hole_offset, 63 - hole_offset, -0.05]) cylinder(r=3.7/2, h=1.7);
        translate([94 - hole_offset, hole_offset, -0.05]) cylinder(r=3.7/2, h=1.7);
        translate([94 - hole_offset, 63 - hole_offset, -0.05]) cylinder(r=3.7/2, h=1.7);
    }
}

module small_breadboard() {
    center = 33 / 2;
    x_offset = (46 - 16 * 2.54) / 2;
    y_offset = 2.54 + 2.54 / 2;
    hole_offset = 4;

    difference() {
        union() {
            color("red") cube([46, 33, 1.6]);
            for (y = [0:4]) {
                translate([x_offset, center + y_offset + y * 2.54, 0]) breadboard_row(17);
                translate([x_offset, center - y_offset - y * 2.54, -0.005]) breadboard_row(17);
            }
        }

        translate([hole_offset, center, -0.05]) cylinder(r=1.6, h=1.7);
        translate([46 - hole_offset, center, -0.05]) cylinder(r=1.6, h=1.7);
    }
}

wall_thickness = 3;
robot_height = 100;
robot_width = 100;

module back_side() {
    union() {
        cube([robot_width, robot_height, wall_thickness]);
        translate([7, 93, wall_thickness - 0.01]) color("gray") m3_screw_in_hole(8);
        translate([7, 38, wall_thickness - 0.01]) color("gray") m3_screw_in_hole(8);
        translate([93, 93, wall_thickness - 0.01]) color("gray") m3_screw_in_hole(8);
        translate([93, 38, wall_thickness - 0.01]) color("gray") m3_screw_in_hole(8);
    }
}

$fn=360;
//wheel_tire();
//wheel(spokes=6);
//right_plate();

//back_side();
//small_breadboard();
back_side();

translate([3, 34, 11.1]) medium_breadboard();

