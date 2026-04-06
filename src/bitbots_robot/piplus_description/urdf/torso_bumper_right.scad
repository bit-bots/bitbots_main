% scale(1000) import("torso_bumper_right.stl");

// Append pure shapes (cube, cylinder and sphere), e.g:
// cube([10, 10, 10], center=true);
// cylinder(r=10, h=10, center=true);
// sphere(10);
translate([-19,0,0]){
translate([19/2, 73/2, 13/2]) cube([19, 73, 13], center=true);

translate([0,36.50000,40+3]){
    rotate([180,0,0]){
    rotate([180-140.582606, 0, 0]){
        translate([19/2, 46.87946/2, 5/2]) cube([19, 46.87946, 5], center=true);
    }
}
}



translate([0,-sin(140.582606)*5,cos(140.582606)*5]){
   translate([0,36.50000,40+3]){
        rotate([180,0,0]){
        rotate([140.582606, 0, 0]){
            translate([19/2, 46.87946/2, 5/2]) cube([19, 46.87946, 5], center=true);
        }
}
}
}

}
