% scale(1000) import("foot_cover.stl");

// Append pure shapes (cube, cylinder and sphere), e.g:
// cube([10, 10, 10], center=true);
// cylinder(r=10, h=10, center=true);
// sphere(10);
translate([-38.391-20, -8.457, 38.166-11]){
    cube([40, 106, 19], center=true);
}
translate([-123.166+(45/2) , -8.457, 38.166-15]){
    rotate([0, 151.928,0]){
    cube([51, 74, 2], center=true);
    }
}