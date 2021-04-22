% scale(1000) import("torso_top.stl");

// Append pure shapes (cube, cylinder and sphere), e.g:
// cube([10, 10, 10], center=true);
// cylinder(r=10, h=10, center=true);
// sphere(10);

translate([0,-135,0]){
    cube([150, 125, 115]);
}
translate([0,-135,5]){
    rotate([180-174.709919,0,0]) {
        cube([150, 10, cos(180-174.709919)*(60-5)]);
    }
}

translate([0,-135-5,60]){
    cube([150, 10, 65.45031]);
}

translate([0,-135-5,115]){
    cube([150, 136, 150-115]);
}