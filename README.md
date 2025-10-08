# RO_012 - TP Filtrage de Kalman

## Q1


Le code étudié met en œuvre un filtre de Kalman étendu (EKF) appliqué à la localisation d’un robot mobile à partir de mesures d’odométrie et d’observations de balises fixes. Il est organisé en plusieurs parties qui reproduisent la chaîne complète d’un système de navigation robotique. Tout d’abord, la partie de simulation du véhicule représente le « monde réel » : elle calcule la trajectoire vraie du robot à partir des commandes de mouvement et y ajoute des bruits afin de simuler les incertitudes des capteurs. Cette section gère aussi la génération de données d’odométrie et de mesures de balises bruitées. Ensuite, le code contient les modèles mathématiques du mouvement et de l’observation, qui décrivent respectivement comment le robot se déplace et comment il perçoit les balises en fonction de sa position. Ces équations sont linéarisées à chaque étape à l’aide des Jacobiennes, indispensables au fonctionnement du filtre de Kalman étendu.

La boucle principale du programme exécute, à chaque pas de temps, la prédiction de la position du robot à partir de l’odométrie, puis la correction de cette estimation dès qu’une mesure de balise est disponible. Le filtre utilise pour cela les matrices de covariance des bruits de mouvement et de mesure (Q et R), ainsi que la covariance de l’estimation P, qui reflète l’incertitude du robot sur sa position. Enfin, le code affiche et compare les trajectoires réelle, odométrique et estimée, permettant d’évaluer la performance du filtre. En résumé, le programme est structuré autour de deux volets principaux : la simulation du comportement physique du robot et de ses capteurs, et l’algorithme d’estimation du filtre de Kalman étendu, qui combine prédiction et correction pour améliorer la précision de la localisation.

## Q2

<p align="center">
  <img src="Q2.png" alt="EKF Trajectory" title="Estimated vs True Trajectory (EKF)" width="500">
</p>

## Q3

![EKF Trajectory](Q3-dt_meas=50.png)
![EKF Trajectory](Q3-dt_meas=100.png)
![EKF Trajectory](Q3-dt_meas=200.png)

## Q4

![EKF Trajectory](Q4-QEst=50.png)
![EKF Trajectory](Q4-QEst=100.png)
![EKF Trajectory](Q4-QEst=200.png)

## Q5

![EKF Trajectory](Q5-REst=50.png)
![EKF Trajectory](Q5-REst=100.png)
![EKF Trajectory](Q5-REst=200.png)

## Q6

![EKF Trajectory](Q6.png)

## Q7

![EKF Trajectory](Q7-n=4.png)
![EKF Trajectory](Q7-n=100.png)
![EKF Trajectory](Q7-n=200.png)

## Q8

![EKF Trajectory](Q8-n=4.png)
![EKF Trajectory](Q8-n=100.png)
![EKF Trajectory](Q8-n=200.png)

## Q9

![EKF Trajectory](Q9-n=4.png)
![EKF Trajectory](Q9-n=100.png)
![EKF Trajectory](Q9-n=200.png)

