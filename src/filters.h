#ifndef filter_h
#define filter_h

/* IIR_WA (Infinite impulse response weighted average)
 * last_filtered  - the last filtered value in the series
 * current        - the current unfiltered value to be last_filtered
 * alpha          - current value weight (between 0 and 1)
 * Returns single filtered value. */
template <class val>
val iir_wa(val last_filtered, val current, double alpha){
  val filtered = alpha*current + (1-alpha)*last_filtered;
  return filtered;
}

/* FIR_MA (Finite impulse response moving average)
 * unfiltered - array of the last n unfiltered items
 * n          - the number of items averaged
 * Returns single filtered value. */
template <class val>
val fir_ma(val unfiltered[], int n){
  val sum;
  for(int i = 0; i < n; i++){
    sum += unfiltered[i];
  }
  val filtered = sum/n;
  return filtered;
}

#endif
