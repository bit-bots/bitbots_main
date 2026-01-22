% scale(1000) import("lense.stl");

// Append pure shapes (cube, cylinder and sphere), e.g:
// cube([10, 10, 10], center=true);
// sphere(10);
rotate([90, 0, 0]){
    translate([0,0,16.2/2]){
        cylinder(r=43/2, h=16.2, center=true);
    }
    translate([0,0,16.2+12/2]){
        cylinder(r=33.5/2, h=12, center=true);
    }
}