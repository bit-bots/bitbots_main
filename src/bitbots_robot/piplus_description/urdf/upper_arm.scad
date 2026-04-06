% scale(1000) import("upper_arm.stl");

// Append pure shapes (cube, cylinder and sphere), e.g:
// cube([10, 10, 10], center=true);
// cylinder(r=10, h=10, center=true);
// sphere(10);
translate([0,-2.25/2-0.25000,0]){
rotate([90,0,0]){
    cylinder(r=20, h=2.25, center=true);
}
}


translate([0,-2.25/2-0.25000,130]){
rotate([90,0,0]){
    cylinder(r=20, h=2.25, center=true);
}
}

translate([0,-2.25/2-0.25000,130/2]){
    cube([40, 2.25, 130], center=true);
}