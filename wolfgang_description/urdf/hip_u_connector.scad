% scale(1000) import("hip_u_connector.stl");

// Append pure shapes (cube, cylinder and sphere), e.g:
// cube([10, 10, 10], center=true);
// cylinder(r=10, h=10, center=true);
// sphere(10);

translate([0,0,18]){
rotate([0,90,90]){
    
    translate([0,0,2]){
    cube([36,100,4], center=true);
    }

    translate([0,50-2,53/2]){
    cube([26.02287,4,53], center=true);
    }
    translate([0,-(50-2),53/2]){
    cube([26.02287,4,53], center=true);
    }
    cylinder(r=25, h=4);
}
}