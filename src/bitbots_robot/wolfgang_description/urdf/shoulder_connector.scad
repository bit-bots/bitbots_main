% scale(1000) import("shoulder_connector.stl");

// Append pure shapes (cube, cylinder and sphere), e.g:
// cube([10, 10, 10], center=true);
// cylinder(r=10, h=10, center=true);
// sphere(10);

translate([0,1.5/2,0]) {
    cube([63.90000, 1.5, 34], center=true);
}

translate([63.9/2-1.5/2,45/2,0]) {
    cube([1.5, 45, 28], center=true);
}
translate([-(63.9/2-1.5/2),45/2,0]) {
    cube([1.5, 45, 28], center=true);
}