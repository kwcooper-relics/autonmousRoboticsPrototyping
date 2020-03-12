/**************************************************************************
*           HAILSTONE
*  Runs the hailstone function.
*
***************************************************************************/

int hailstone(int n);

task main() {
	//start at 7
	int n = 7;
	//stop sequence when it hits 1
	while ( n != 1) {
		writeDebugStreamLine("n = %d\n",n);
		//run hailstone and save results as n
		n = hailstone(n);
	}
	//print final value
	writeDebugStreamLine("n = %d\n",n);
}

//computes n_{i+1} given n_i
int hailstone(int n){
		if (n % 2 == 0) {
			return n / 2;
		}	else {
			return 3 * n + 1;
		}
}
