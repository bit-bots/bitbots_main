% scale(1000) import("motor_connector.stl");

// Append pure shapes (cube, cylinder and sphere), e.g:
translate([0,-6,0]){
cube([28, 6, 5.7]);
}
translate([0,-6,5.7]){
cube([28, 3.5, 5.9]);
}
// cylinder(r=10, h=10, center=true);
// sphere(10);
