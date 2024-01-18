% scale(1000) import("camera_lower_basler_wolfgang_imu_v2.2.stl");

// Append pure shapes (cube, cylinder and sphere), e.g:

translate([-15.55499+3,-72/2,18-30/2]){
    cube([6, 72, 30], center=true);
    
    translate([67,0,0]){
    cube([6, 72, 30], center=true);
    }
}
// cylinder(r=10, h=10, center=true);
// sphere(10);
translate([73/2 - 15.55499,-3.5,18-30/2]){
cube([73, 7, 30], center=true);
}
translate([45/2-8.05499,22/2-7,55/2-12]){
cube([45, 22, 55], center=true);
}