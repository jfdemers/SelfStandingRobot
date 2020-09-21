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

$fn=360;
wheel_tire();
//wheel(spokes=6);
