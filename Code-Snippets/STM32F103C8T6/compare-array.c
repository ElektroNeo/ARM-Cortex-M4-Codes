/* Function to compare array elements */
uint8_t compareArray(char a[], char b[], uint8_t size)	{
	for(uint8_t i=0; i<size; i++){
		if(a[i]!=b[i])
			return (0);
	}
	return (1);
}