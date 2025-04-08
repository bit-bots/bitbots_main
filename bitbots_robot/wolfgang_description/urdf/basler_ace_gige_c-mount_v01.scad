% scale(1000) import("basler_ace_gige_c-mount_v01.stl");

// Append pure shapes (cube, cylinder and sphere), e.g:
translate([0,-(42/2+12),0]){
cube([29, 42, 29], center=true);
}
translate([0,-6,0]){
rotate([90,0,0]){
cylinder(r=14, h=12, center=true);
}
}
// sphere(10);
