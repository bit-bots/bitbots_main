% scale(1000) import("knee_connector.stl");

// Append pure shapes (cube, cylinder and sphere), e.g:
translate([0,17.98262/2-4,-(24.5/2-30.34724)]){
cube([42, 17.98262, 24.5], center=true);
}

translate([0,24.81250/2-4,-(4/2-30.34724)]){
cube([34, 24.81250, 4], center=true);
}
// cylinder(r=10, h=10, center=true);
// sphere(10);
