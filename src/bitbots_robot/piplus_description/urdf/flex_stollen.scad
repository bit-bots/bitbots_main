% scale(1000) import("flex_stollen.stl");

// Append pure shapes (cube, cylinder and sphere), e.g:
// cube([10, 10, 10], center=true);
translate([0,0,5]){
 cylinder(r=6, h=10, center=true);
}
sphere(6);
