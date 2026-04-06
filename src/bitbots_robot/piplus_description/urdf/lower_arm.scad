% scale(1000) import("lower_arm.stl");

// Append pure shapes (cube, cylinder and sphere), e.g:
// cube([10, 10, 10], center=true);
rotate([90,0,0]){
    translate([0,0,1]){
        cylinder(r=40, h=2, center=true);
    }
}
forearm_len = 145;
forearm_start = 28;
translate([-1.5,-1,-(28+145/2)]){
    cube([44,2,145], center=true);
}

// sphere(10);
translate([0,-1,30]){
rotate([90,0,0]){
        cylinder(r=32, h=2, center=true);
}
}

translate([35-6.8,-1,24]){
    rotate([0, 165.522,0]){
        cube([12,2,29], center=true);
    }
}
translate([-35+6.8,-1,24]){
    rotate([0, -165.522,0]){
        cube([12,2,29], center=true);
    }
}