% scale(1000) import("hand.stl");

// Append pure shapes (cube, cylinder and sphere), e.g:
// cube([10, 10, 10], center=true);
// cylinder(r=10, h=10, center=true);
// sphere(10);

// Append pure shapes (cube, cylinder and sphere), e.g:
// cube([10, 10, 10], center=true);
rotate([90,0,0]){
    translate([0,0,17]){
        cylinder(r=40, h=34, center=true);
    }
}

// sphere(10);
translate([0,-17,30]){
rotate([90,0,0]){
        cylinder(r=32, h=34, center=true);
}
}

translate([35-6.8,-17,24]){
    rotate([0, 165.522,0]){
        cube([12,34,29], center=true);
    }
}
translate([-35+6.8,-17,24]){
    rotate([0, -165.522,0]){
        cube([12,34,29], center=true);
    }
}