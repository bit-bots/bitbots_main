% scale(1000) import("lower_leg.stl");

// Append pure shapes (cube, cylinder and sphere), e.g:
// cube([10, 10, 10], center=true);
translate([0,-1,0]){
rotate([90,0,0]){
    cylinder(r=18, h=2, center=true);
}
}
translate([0,-1,170]){
    rotate([90,0,0]){
        cylinder(r=18, h=2, center=true);
    }
}

translate([-4,-1,5]){
rotate([90,0,0]){
    cylinder(r=20, h=2, center=true);
}
}

translate([4,-1,165]){
rotate([90,0,0]){
    cylinder(r=20, h=2, center=true);
}
}

translate([-24,-2,6]){
    rotate([0,5,0]){
        translate([34/2,2/2,160/2]) {
            cube([34, 2, 160], center=true);
        }
    }
}

// cylinder(r=10, h=10, center=true);
// sphere(10);
