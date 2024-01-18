% scale(1000) import("springholder_new.stl");

// Append pure shapes (cube, cylinder and sphere), e.g:
// cube([10, 10, 10], center=true);
// cylinder(r=10, h=10, center=true);
// sphere(10);

translate([0,-22+3/2,170]){
    rotate([90,0,0]){
        cylinder(r=33/2, h=3, center=true);
    }
}

translate([0,-22+16.3/2+3,170]){
    rotate([90,0,0]){
        cylinder(r=21/2, h=16.3, center=true);
    }
}