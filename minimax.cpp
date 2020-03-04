// parameters : 
// grid has 9 cases in order to play Tic Tac Toe
// emptyCases contains the indices of the empty cases in grid
// player can be either COMPUTER or HUMAN
// firstIteration must be set as true when minimax function is called outside of the minimax funtiob
// bestMoveOrdi will contain the index of the case to play
//
// A case in grid can have 3 values : EMPTY, COMPUTER, HUMAN
// The function winningGrid returns true if the tested player has managed to get a complete row, column or diagonal
//
int minimax( vector<int> grid, vector<int> emptyCases, int player, bool firstIteration, int &bestMoveOrdi )
{
	if( winningGrid( grid, COMPUTER ) ) return 1;
	if( winningGrid( grid, HUMAN ) ) return -1;
	if( emptyCases.size() < 1 ) return 0;

	int best = player == COMPUTER ? -10 : 10;

	for( int i = 0; i < emptyCases.size(); ++i ) {
		int indexMove = emptyCases[ i ];
		grid[ indexMove ] = player;
		emptyCases.erase( emptyCases.begin() + i );

		int resMinMax = minimax( grid, emptyCases, player == COMPUTER ? HUMAN : COMPUTER, false, bestMoveOrdi );

		if( player == COMPUTER && resMinMax > best) {
			best = resMinMax;
			if( firstIteration ) bestMoveOrdi = indexMove;
		}
		if( player == HUMAN && resMinMax < best) {
			best = resMinMax;
		}

		emptyCases.insert( emptyCases.begin()+i, indexMove );
		grid[ indexMove ] = EMPTY;
	}

	return best;
}
