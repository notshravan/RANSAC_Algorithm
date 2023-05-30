Name: Shravan Vyas
Student Number: 300089045
Course: CSI 2120, Winter 2023

Project Part 3 Scheme
All of the code for this part is contained within the file "SchemePart3".
A complete explanation of the code is added within the comments, my implementation differs abit from the guidelines.

This submission contains 8 files in total.

Usage: (plane-ransac "Point_Cloud_x_No_Road_Reduced.xyz" Confidence Percentage Eps),
where 'x' is the file number

Parameters:
I chose:
Confidence = 0.99
Percentage = 0.8
Eps = 1

Example Prompt: > (plane-ransac "Point_Cloud_1_No_Road_Reduced.xyz" 0.99 0.8 1)
This will return the Best-Plane equation, Best Support(%) Pair.

[Output Results : ]
[NOTE] (Below contains the results received, For a FULL EXPLANATION, Visit the files DominantPlane1, DominantPlane 2 & DominantPlane3 as they contain a full interpretation of the results)

"Point_Cloud_1_No_Road_Reduced.xyz" :- '((-38.049596746669444 317.25070729455103 249.73979513693587 506.0220962084952) . 1295)

Explanation found in file: DominantPlane1.txt

"Point_Cloud_2_No_Road_Reduced.xyz" :- '((127.56378083536654 -202.79672446512345 29.39294827560541 -1892.847509022202) . 431)
Explanation found in file: DominantPlane2.txt

"Point_Cloud_3_No_Road_Reduced.xyz" :- '((229.94989483757064 -214.67214789636765 154.76452320423837 1042.5808362279174) . 2376)
Explanation found in file: DominantPlane3.txt




