mkdir -p selected
mkdir -p disparities
mv right* selected/
mv left* selected/
mv disp*.png disparities/


mv disparities ~/AfstudeerData/DroneCam/$1/
mv selected ~/AfstudeerData/DroneCam/$1/
mv export.txt ~/AfstudeerData/DroneCam/$1/