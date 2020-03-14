#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <queue>
#include <chrono> // std::chrono::microseconds
#include <thread> // std::this_thread::sleep_for;


using namespace std;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//									GOAL ORIENTED ACTION PLANNING (GOAP)
//
// L'IA est composé de :
//   - un état contenant plusieurs variables représentant les informations disponibles sur le monde
//   - des actions permettant de modifier l'état
//
// Pour chaque action, on doit définir :
//   - une fonction pré-condition qui renvoie l'état initial désiré pour pouvoir lancer l'action
//   - une fonction résultat qui simule la modification de l'état initial une fois que l'action sera lancée, en fonction de l'état voulu
//   - une fonction pour calculer le coût de l'action
//   - une fonction "tick" qui sera appelée par le système afin de modifier l'état courant de façon itérative (= ce que doit faire l'action pendant un tick)
//
// Le but du GOAP est de trouver la suite d'actions à faire pour partir d'un état initial et arriver à état voulu.
// Si plusieurs possibilités existent, on choisit celle avec le plus petit coût total
//
//
// @torea Foissotte, Poly3D, 28/02/2020
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//									LE SYSTEME INTERNE
//
// pas besoin de modifier ça normalement...


// une classe qui contient toutes les variables utilisées par l'IA pour faire ses actions
class Etat {
public:
	map<string, int> infos;
};

// Definit comment on différencie 2 états
// On prend l'hypothèse qu'on comparera généralement l'état courant (avec toutes les variables présentes) 
// avec un état intermédiaire contenant seulement quelques variables nécessaires
// 
// état l est différent de état r si :
//  - au moins une des variables qui existe dans les 2 tableaux a des valeurs différentes. Par exemple : l.infos["vie"]=10 et r.infos["vie"]=3
//  - ou si les 2 tableaux n'ont aucunes variables en commun
bool operator!=( const Etat& l, const Etat& r )
{
	int numCommun = 0;
	for( auto itr = l.infos.begin(); itr != l.infos.end(); ++itr ) {
		if( r.infos.find( itr->first ) == r.infos.end() ) continue;
		numCommun++;
		if( itr->second != r.infos.at( itr->first ) ) return true;
	}
	if( numCommun == 0 ) return true;
	return false;
}
bool operator==( const Etat& l, const Etat& r )
{
	return !(l!=r);
}

// fonction utilitaire pour afficher l'état complet avec un cout
std::ostream& operator <<( std::ostream& stream, const Etat& etat ) {
	for( auto itr = etat.infos.begin(); itr != etat.infos.end(); ++itr ) {
		stream << itr->first << " : " << itr->second << endl;
	}
	return stream;
}


// la classe de base dont chaque action doit heriter
class actionInterface {
public:
	// il faudra redéfinir ces 4 fonctions dans les classes dérivées
	virtual Etat preConditions( const Etat& etatInitial, const Etat& etatVoulu ) { return etatInitial; };
	virtual Etat resultats( const Etat& etatInitial, const Etat& etatVoulu ) { return etatVoulu; };
	virtual int coutAction( Etat& etatInitial, const Etat& etatVoulu ) { return 1000; };
	virtual bool action_tick( Etat& etatInitial, const Etat& etatVoulu ) { return true; };

	// pas besoin de redéfinir cette fonction-là
	Etat etatResultatTotal( const Etat& etatInitial, const Etat& etatVoulu ) {
		Etat total = etatInitial;
		Etat res = resultats( etatInitial, etatVoulu );
		for( auto itr = res.infos.begin(); itr != res.infos.end(); ++itr ) {
			total.infos[itr->first] = itr->second;
		}
		return total;
	}
};

// Un tableau qui contiendra toutes les actions possibles pour notre IA
// toutes les classes dérivées de actionInterface seront placées dedans
map<string, actionInterface*> actions;

// La fonction Tick à appeler régulièrement pour exécuter toutes les actions présentes dans la séquence
bool Tick( vector<string>& sequenceActions, Etat& etatActuel, const Etat& etatVoulu )
{
	static string actionEnCours = "";	// stockera l'action actuellement en cours d'excution dans le tick
	cout << "tick.." << endl;
	if( actionEnCours == "" ) {
		// si pas d'action en cours, on verifie s'il y a encore des actions à faire dans la file d'attente
		if( sequenceActions.size() < 1 ) return false;	// il n'y a plus d'actions à faire

		// récupère la prochaine action à faire
		actionEnCours = sequenceActions.front();
		sequenceActions.erase( sequenceActions.begin() );
	}

	// effectue un action_tick de l'action en cours
	// si action_tick renvoie true, l'action est finie, on passera a une autre action le tick suivant
	if( actions[actionEnCours]->action_tick( etatActuel, etatVoulu ) ) actionEnCours = "";

	return true;
}


// la classe pour stocker une séquence d'actions, un etat a la fin de la sequence et un score total
struct Sequence {
	vector<string> seq;
	Etat etaTmp;
	int score;

	Sequence( vector<string> sq, Etat e, int s ) {
		seq = sq;
		etaTmp = e;
		score = s;
	};
	Sequence() : score(0) {};

	void reset() { seq.clear(); etaTmp = Etat(); score = 0; };
};

// une fonction pour ordonner 2 séquences. Utilisé pour trouver la sequence avec le score minimum
bool cmpSequence( Sequence s1, Sequence s2 )
{
	return s1.score < s2.score;
}

// la fonction principale du GOAP!
// utilise A* pour trouver la séquence d'action avec le score minimum pour aller de l'état initial à l'état voulu
bool calculePlanPourAtteindreEtat( const Etat& etatDepart, Etat etatVoulu, Sequence& seqResultat )
{
	if( etatDepart == etatVoulu ) return true;	// rien à faire si on demande d'atteindre l'état actuel

	vector<string> vs;
	vector<Sequence> sequencesPossibles;		// pour stocker toutes les séquences possibles
	sequencesPossibles.push_back( Sequence( vs, etatDepart, 0 ) );

	while( 1 ) {
		// prend l'element de sequencesPossibles qui a le plus petit score
		auto seqMinItr = min_element( sequencesPossibles.begin(), sequencesPossibles.end(), cmpSequence );
		Sequence seq = *(seqMinItr);

		if( seq.etaTmp == etatVoulu ) {
			seqResultat = seq;
			return true;
		}

		// on le sort du tableau
		sequencesPossibles.erase( seqMinItr );

		// cherche la premiere action qui peut se lancer depuis l'etat final de la sequence
		for( auto a = actions.begin(); a != actions.end(); ++a ) {
			if( a->second->preConditions( seq.etaTmp, etatVoulu ) != seq.etaTmp ) continue;
			// l'action peut etre demarree avec l'etat de depart!

			// recupere le cout de cette action
			int scoreAction = a->second->coutAction( seq.etaTmp, etatVoulu );

			// cree une nouvelle suite d'action = suite d'actions de la sequence + action courante
			vs = seq.seq;
			vs.push_back( a->first );

			// cree une nouvelle sequence avec la nouvelle suite d'action, l'etat obtenu apres cette action, et le score
			Sequence nSeq( vs, a->second->etatResultatTotal( seq.etaTmp, etatVoulu ), seq.score + scoreAction );

			// on place la sequence dans la liste
			sequencesPossibles.push_back( nSeq );
		}

		if( sequencesPossibles.size() < 1 ) return false;	// pas de sequence a tester : impossible de trouver une solution!
	}

	return false;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//									DEFINITION DE NOTRE IA
//
// A modifier, supprimer, ajouter, ...


// Tous les etats que l'on pourra mettre dans Etat.infos :
const string N_ETAGE = "Numero Etage";
const string PORTE_OUV = "Portes Ouvertes";


// rajouter plusieurs classes héritant de actionInterface, 
// une classe par action exécutable par notre IA

class actionOuvrePorte : actionInterface {
public:
	Etat preConditions( const Etat& etatInitial, const Etat& etatVoulu ) {
		Etat etatPossible;
		etatPossible.infos[PORTE_OUV] = 0;		// on ne peut ouvrir la porte que si elle est fermee
		return etatPossible;
	}
	Etat resultats( const Etat& etatInitial, const Etat& etatVoulu ) {
		Etat etatFinal;
		etatFinal.infos[PORTE_OUV] = 1;			// a la fin de l'action le seul etat qui aura changé est celui de la porte
		return etatFinal;
	}
	int coutAction( Etat& etatInitial, const Etat& etatVoulu ) { 
		return 1;	// c'est facile d'ouvrir une porte donc on renvoie un petit nombre
	};

	bool action_tick( Etat& etatCourant, const Etat& etatVoulu ) {
		// l'action se contente juste d'afficher son nom et de modifier l'etat courant
		std::cout << "action Ouvre Porte" << endl;
		etatCourant.infos[PORTE_OUV] = 1;	// = la porte est ouverte maintenant
		return true;		// l'action est finie
	};
};

class actionFermePorte : actionInterface {
public:
	Etat preConditions( const Etat& etatInitial, const Etat& etatVoulu ) {
		Etat etatPossible;
		etatPossible.infos[PORTE_OUV] = 1;		// on ne peut fermer la porte que si elle est ouverte
		return etatPossible;
	}
	Etat resultats( const Etat& etatInitial, const Etat& etatVoulu ) {
		Etat etatFinal;
		etatFinal.infos[PORTE_OUV] = 0;
		return etatFinal;
	}
	int coutAction( Etat& etatInitial, const Etat& etatVoulu ) {
		return 1;
	};

	bool action_tick( Etat& etatCourant, const Etat& etatVoulu ) {
		std::cout << "action Ferme Porte" << endl;
		etatCourant.infos[PORTE_OUV] = 0;
		return true;
	};
};

class actionBouge : actionInterface {
public:
	Etat preConditions( const Etat& etatInitial, const Etat& etatVoulu ) {
		Etat pc;
		// on peut bouger l'ascenseur seulement si 
		// - la porte est fermee
		// - l'etage ou on veut aller est different de l'etage d'ou on part
		//   si les etages de depart et arrivee sont egaux, on met un n° etage different de l'etat initial pour que la pre-condition ne soit pas satisfaite
		pc.infos[PORTE_OUV] = 0;
		if( etatInitial.infos.at( N_ETAGE ) == etatVoulu.infos.at( N_ETAGE ) ) pc.infos[N_ETAGE] = etatInitial.infos.at( N_ETAGE ) - 1;
		return pc;
	}
	Etat resultats( const Etat& etatInitial, const Etat& etatVoulu ) {
		Etat pc;
		pc.infos[N_ETAGE] = etatVoulu.infos.at( N_ETAGE );
		return pc;
	}
	int coutAction( Etat& etatInitial, const Etat& etatVoulu ) {
		// le cout correspond au nombre d'etage entre le depart et l'arrivee
		int score = etatInitial.infos.at( N_ETAGE ) - etatVoulu.infos.at( N_ETAGE );
		return score<0?-score:score;
	};

	bool action_tick( Etat& etatCourant, const Etat& etatVoulu ) {
		std::cout << "action Bouge" << endl;
		if( etatCourant.infos[N_ETAGE] < etatVoulu.infos.at( N_ETAGE ) ) etatCourant.infos[N_ETAGE]++;
		else if( etatCourant.infos[N_ETAGE] > etatVoulu.infos.at( N_ETAGE ) ) etatCourant.infos[N_ETAGE]--;
		
		if( etatCourant.infos[N_ETAGE] != etatVoulu.infos.at( N_ETAGE ) ) return false;
		return true;
	};
};


// placer toutes les actions définies au dessus dans le tableau actions
void initAction()
{
	actions["ouvre"] = (actionInterface*)(new actionOuvrePorte());
	actions["ferme"] = (actionInterface*)(new actionFermePorte());
	actions["bouge"] = (actionInterface*)(new actionBouge());
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//								PETIT PROGRAMME EXEMPLE POUR TESTER L'IA
//
// A modifier

// point d'entrée du programme
// quand l'exécutable est lancé, c'est la fonction qui est appelée au début
int main()
{
	initAction();

	Etat etatCourant;
	etatCourant.infos[N_ETAGE] = 0;
	etatCourant.infos[PORTE_OUV] = 1;

	cout << "Etat courant : " << endl << etatCourant << endl;

	cout << "On veut atteindre l'etage 4 - PO" << endl;
	Etat etatVoulu;
	etatVoulu.infos[N_ETAGE] = 4;
	etatVoulu.infos[PORTE_OUV] = 1;

	Sequence planActions;
	calculePlanPourAtteindreEtat( etatCourant, etatVoulu, planActions );
	cout << endl << "plan : "<< endl;
	for( auto it = planActions.seq.begin(); it != planActions.seq.end(); ++it ) cout << " - "<< *it << endl;
	while( Tick( planActions.seq, etatCourant, etatVoulu ) ) this_thread::sleep_for( 1s );

	cout << "Maintenant on veut atteindre l'etage -2 - PO" << endl;
	planActions.reset();
	etatVoulu.infos[N_ETAGE] = -2;
	etatVoulu.infos[PORTE_OUV] = 1;
	calculePlanPourAtteindreEtat( etatCourant, etatVoulu, planActions );
	while( Tick( planActions.seq, etatCourant, etatVoulu ) ) this_thread::sleep_for( 1s );


	cout << "Fini!";
	return 0;
}

