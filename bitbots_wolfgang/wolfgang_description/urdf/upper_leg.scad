% scale(1000) import("upper_leg.stl");

// Append pure shapes (cube, cylinder and sphere), e.g:
// cube([10, 10, 10], center=true);
// cylinder(r=10, h=10, center=true);
// sphere(10);
translate([0,-1,0]){
rotate([90,0,0]){
    cylinder(r=18, h=2, center=true);
}
}

/*translate([19.32474,0,-25.75647]){
    rotate([90,0,0]){
        cylinder(r=50, h=2, center=true);
    }
}*/

translate([9.329,-1,-143]){
    rotate([90,0,0]){
        cylinder(r=10, h=2, center=true);
    }
}
translate([14.84033,-1,-132.65892]){
    rotate([90,0,0]){
        cylinder(r=20, h=2, center=true);
    }
}
//38, 2,sqrt(pow(108.94147,2)-pow(38,2))
translate([34.15884-cos(-15)*38,-2,-127.48254+sin(-15)*38]){
    rotate([0,-15,0]){
        translate([38/2, 1, sqrt(pow(108.94147,2)-pow(38,2))/2]) {
        cube([38, 2,sqrt(pow(108.94147,2)-pow(38,2))], center=true);
        }
    }
}

translate([5.15043,-2,-19.22168]){
    rotate([0,-15+180,0]){
        translate([10/2,2/2,10/2]){
            cube([10,2,10], center=true);
        }
    }
}

translate([9.329,-1,-143]){
    rotate([90,0,0]){
        cylinder(r=10, h=2, center=true);
    }
}

num_circ = 10;
r1 = 50;
r2 = 17;
step = 3;
initial_angle = 45;

translate([19.32474-cos(initial_angle)*(r1-r2),
           -1,
           -25.75647+sin(initial_angle)*(r1-r2)]){
    rotate([90,0,0]){
        cylinder(r=r2, h=2, center=true);
    }
}
translate([19.32474-cos(initial_angle-10)*(r1-r2),
           -1,
           -25.75647+sin(initial_angle-10)*(r1-r2)]){
    rotate([90,0,0]){
        cylinder(r=r2, h=2, center=true);
    }
}
translate([19.32474-cos(initial_angle-20)*(r1-r2),
           -1,
           -25.75647+sin(initial_angle-20)*(r1-r2)]){
    rotate([90,0,0]){
        cylinder(r=r2, h=2, center=true);
    }
}
translate([19.32474-cos(initial_angle-30)*(r1-r2),
           -1,
           -25.75647+sin(initial_angle-30)*(r1-r2)]){
    rotate([90,0,0]){
        cylinder(r=r2, h=2, center=true);
    }
}
translate([19.32474-cos(initial_angle-40)*(r1-r2),
           -1,
           -25.75647+sin(initial_angle-40)*(r1-r2)]){
    rotate([90,0,0]){
        cylinder(r=r2, h=2, center=true);
    }
}
translate([19.32474-cos(initial_angle-50)*(r1-r2),
           -1,
           -25.75647+sin(initial_angle-50)*(r1-r2)]){
    rotate([90,0,0]){
        cylinder(r=r2, h=2, center=true);
    }
}
translate([19.32474-cos(initial_angle-60)*(r1-r2),
           -1,
           -25.75647+sin(initial_angle-60)*(r1-r2)]){
    rotate([90,0,0]){
        cylinder(r=r2, h=2, center=true);
    }
}

