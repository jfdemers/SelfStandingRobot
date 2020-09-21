// Create a tire with the specified radius r, height h and thread height th.
module tire(r=50, h=10, th=2) {
    perimeter = 2 * PI * r;
    thread_spacing = 2 * 360 / perimeter;
    
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

module wheel(r=50, h=10) {
    difference() {
        union() {
            translate([0,0,2]) {
                union() {
                    cylinder(r=r, h=h);
                    translate([0,0,h]) cylinder(r=r+2, h=2);
                    translate([0,0,-2]) cylinder(r=r+2, h=2);
                }
            }
        }

        linear_extrude(height=10) {
            
        }
    }
}

$fn=200;
//tire();
wheel();
