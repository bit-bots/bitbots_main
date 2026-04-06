% scale(1000) import("knee_spacer.stl");

// Append pure shapes (cube, cylinder and sphere), e.g:
// cube([10, 10, 10], center=true);
translate([-27.97000, -19.019, 154.130]){
    rotate([90,0,0]){
        cylinder(r=18, h=1.5, center=true);
    }
}
// sphere(10);
