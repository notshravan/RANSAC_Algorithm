 % Read point cloud data from a file
 read_xyz_file(File, Points) :-
     open(File, read, Stream),
     read_xyz_points(Stream, Points),
     close(Stream).

 read_xyz_points(Stream, []) :-
     at_end_of_stream(Stream).

 read_xyz_points(Stream, [Point | Points]) :-
     \+ at_end_of_stream(Stream),
     read_line_to_string(Stream, L),
     split_string(L, "\t", "\s\t\n", XYZ),
     convert_to_float(XYZ, Point),
     read_xyz_points(Stream, Points).

 convert_to_float([], []).
 convert_to_float([H | T], [HH | TT]) :-
     atom_number(H, HH),
     convert_to_float(T, TT).

 % Random3points predicate
 %  This predicate should be true if Point3 is a triplet of points 
 %  randomly selected from the list of points Points. 
 %  The triplet of points is of the form [[x1,y1,z1], [x2,y2,z2], [x3,y3,z3]].
 random3points(Points, Point3) :-
     length(Points, Len),
     random_between(1, Len, I1),
     random_between(1, Len, I2),
     random_between(1, Len, I3),
     nth1(I1, Points, P1),
     nth1(I2, Points, P2),
     nth1(I3, Points, P3),
     Point3 = [P1, P2, P3].

 % Plane predicate
 %This predicate should be true if Plane is the equation of the plane defined by the three points of the list Point3. 
 %The plane is specified by the list [a,b,c,d] from the equation ax+by+cz=d. 
 %The list of points is of the form [[x1,y1,z1], [x2,y2,z2], [x3,y3,z3]].
 plane([[X1, Y1, Z1], [X2, Y2, Z2], [X3, Y3, Z3]], [A, B, C, D]) :-
     A is (Y1-Y2)*(Z1-Z3) - (Z1-Z2)*(Y1-Y3), %performs cross-product and obtains plane equations
     B is (Z1-Z2)*(X1-X3) - (X1-X2)*(Z1-Z3),
     C is (X1-X2)*(Y1-Y3) - (Y1-Y2)*(X1-X3),
     D is - (A*X1 + B*Y1 + C*Z1).

 % Support predicate
 point_plane_distance([A, B, C, D], [X, Y, Z], Distance) :-
     Distance is abs((A*X + B*Y + C*Z + D) / sqrt(A*A + B*B + C*C)).

%This predicate should be true if the support of plane Plane is composed of N points from
%the list of points Point3 when the distance Eps is used.
 support(_, [], _, 0).
 support(Plane, [Point|Points], Eps, N) :-
     point_plane_distance(Plane, Point, Distance),
     support(Plane, Points, Eps, NRest),
     (   Distance =< Eps
     ->  N is NRest + 1
     ;   N is NRest
     ).

 % RANSAC number of iterations predicate
 %This predicate should be true if N is the number of iterations required by RANSAC with parameters Confidence 
 %and Percentage according to the formula given in the problem description section.
 ransac_number_of_iterations(Confidence, Percentage, N) :-
     LogDenom is log(1 - (Percentage ** 3)),
     N is ceil(log(1 - Confidence) / LogDenom).

 % Test cases

 %test case "Point_Cloud_1_No_Road_Reduced.xyz"
 test(Plane,1)
    :- read_xyz_file("Point_Cloud_1_No_Road_Reduced.xyz", Points),
        random3points(Points, Point3),
        plane(Point3, Plane),
        support(Plane, Points, 1, N),
        N > 1000.

    %test case "Point_Cloud_2_No_Road_Reduced.xyz"
    test(Plane,2)
    :- read_xyz_file("Point_Cloud_2_No_Road_Reduced.xyz", Points),
        random3points(Points, Point3),
        plane(Point3, Plane),
        support(Plane, Points, 1, N),
        N > 1000.

    %test case "Point_Cloud_3_No_Road_Reduced.xyz"
    test(Plane,3)
    :- read_xyz_file("Point_Cloud_3_No_Road_Reduced.xyz", Points),
        random3points(Points, Point3),
        plane(Point3, Plane),
        support(Plane, Points, 1, N),
        N > 1000.

    %Result for a success console should return Plane = random3points.


 test(random3points, 1) :-
     random3points([[1, 2, 3], [2, 3, 4], [3, 4, 5]], Point3),
     length(Point3, 3).

 test(plane, 1) :-
     plane([[1, 0, 0], [0, 1, 0], [0, 0, 1]], [-1, -1, -1, 1]).

 test(support, 1) :-
     support([1, 0, -1, 0], [[1, 1, 1], [2, 2, 2], [3, 3, 3]], 0.01, 3).

 test(ransac_number_of_iterations, 1) :-
          ransac_number_of_iterations(0.99, 0.8, 46).

          test(ransac_number_of_iterations, 2) :-
          ransac_number_of_iterations(0.95, 0.8, 3).

          % To run the test cases
          run_tests :-
          test(random3points, N1),
          format("Test random3points w passedn", [N1]),
          test(plane, N2),
          format("Test plane w passedn", [N2]),
          test(support, N3),
          format("Test support w passedn", [N3]),
          test(ransac_number_of_iterations, N4),
          format("Test ransac_number_of_iterations w passedn", [N4]),
          test(ransac_number_of_iterations, N5),
          format("Test ransac_number_of_iterations w passedn", [N5]).

          % Run the tests with the query
          % ?- run_tests.


%Using Results obtained from Scheme.