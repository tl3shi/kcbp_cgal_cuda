# regression tests with program 'bench'

echo "DOP tree"
./bench -n 5 500 -d 1.5 0.501 -T -g to -x 4 -A do
./bench -n 5 500 -d 1.5 0.501 -T -g sh -x 7 -A do
./bench -n 5 500 -d 1.5 0.499 -T -g pl -x 4 -A do

echo ""
echo "Boxtree"
./bench -n 5 500 -d 1.5 0.501 -T -g to -x 4 -A bx
./bench -n 5 500 -d 1.5 0.501 -T -g sh -x 7 -A bx
./bench -n 5 500 -d 1.5 0.499 -T -g pl -x 4 -A bx

