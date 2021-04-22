% scale(1000) import("ankle.stl");

// Append pure shapes (cube, cylinder and sphere), e.g:
// cube([10, 10, 10], center=true);
// cylinder(r=10, h=10, center=true);
// sphere(10);
translate([0,0,2]){
cube([48,100,4], center=true);
}

translate([0,50-2,50/2]){
cube([48,4,50], center=true);
}
translate([0,-(50-2),50/2]){
cube([48,4,50], center=true);
}