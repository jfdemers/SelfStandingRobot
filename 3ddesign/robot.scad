// Create a tire with the specified radius r, height h and thread height th.
module tire(r=50, h=10, th=2) {
    perimeter = 2 * PI * r;
    thread_spacing = 3 * 360 / perimeter;
    
    rotate([0,90,0])
        union() {
            difference() {
                cylinder(r=r, h=h);
                translate([0, 0, -0.5]) cylinder(r=r-2, h=h+1);
            }
            
            for (i = [0:thread_spacing:360]) {
                    rotate([0, 0, i])
                        translate([r-0.5, 0, 0])
                            cube([th, 1.0, h]);
            }
        }
}

module wheel_hole() {
    difference() {
        scale([5, 1]) circle(r=6);
        translate([0, -5]) square([25,10]);
    }
}

module wheel_dent(r, h) {
    rotate_extrude()
        translate([r-2, 0, 0])
            square([3, h-2]);
}

module wheel_design(r, h, hub) {
    c_radius = (r - 2 - hub) / 2 - 3;

    union() {
        for (i=[0:20]) {
            rotate([0, 0, i * 360 / 20])
            translate([hub + 3 + c_radius, 0, -0.5])
                linear_extrude(h+1)
                    scale([1, 0.1])
                        circle(r=c_radius);
        }
    }
}

module wheel_hub(r, h) {
    cylinder(r=r, h=h);
}

module wheel(r=50, h=12, hub=4) {
    rotate([0,90,0])
        difference() {
            cylinder(r=r, h=h);
            translate([0, 0, 1]) wheel_dent(r,h);
            wheel_design(r, h, hub);
            translate([0, 0, -0.5]) wheel_hub(r=hub, h=h+1);
        }
}

module wheel_tire(r=50, h=12, hub=2) {
color("gray") wheel(r=r,h=h,hub=hub);
color("orange")
    translate([1.25,0,0]) tire(r=r, h=h-2.5);
}

$fn=200;

//wheel_tire();
wheel();
//wheel_design(50,12,4);