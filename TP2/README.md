# RO_012 - TP Filtrage Particulaire

## Q1

Le programme met en place un filtre particulaire pour localiser un robot mobile en 2D. Le simulateur crée une trajectoire vraie, calcule l’odométrie bruitée et génère des mesures de capteur à partir d’amers connus. Le modèle de mouvement utilise une commande avec du bruit pour prédire la nouvelle position des particules. Le modèle d’observation calcule la distance et l’angle entre le robot et un amer. À chaque pas de temps, les particules sont déplacées selon la commande, puis pondérées en fonction de la vraisemblance de la mesure. Les poids sont ensuite normalisés. L’estimation de la position du robot est obtenue par la moyenne pondérée des particules. Si le nombre effectif de particules devient trop faible, un rééchantillonnage est effectué pour éviter la dégénérescence. Le code distingue les paramètres réels (QTrue, RTrue) utilisés pour simuler le monde et les paramètres estimés (QEst, REst) utilisés par le filtre. Des fonctions utilitaires gèrent les calculs trigonométriques, les intégrations et l’affichage des trajectoires. Au fil du temps, le filtre ajuste la distribution des particules pour suivre la position réelle du robot.



## Q2

Le fichier ParticleFilter.py a été complété afin d’intégrer l’ensemble des équations du filtre particulaire (PF). Le modèle dynamique, le modèle de mesure ainsi que le prédiction, correction et ré-echantillonnage associées ont été implémentés conformément aux équations du cours.

Le résultat de la simulation avec les données initiales peut être observé dans la figure suivante.


<p align="center">
  <img src="Q2.png" alt="PF Trajectory" title="Estimated vs True Trajectory (PF)" width="500">
  <br>
  <em>Figure 1 – Résultat Initial </em>
</p>

Pour cette simulation, on peut constater que la trajectoire réelle et la trajectoire du filtre sont très proches, pratiquement identiques, avec une erreur faible.


## Q3

Ici on peut voir des simulations en faisant varier le bruit dynamique du filtre (QEst):

<p align="center">
  <img src="Q3-QEst=50.png" alt="" title="" width="500">
  <br>
  <em>Figure 2 – Variation du bruit dynamique du filtre QEst = 50 </em>
</p>

<p align="center">
  <img src="Q3-QEst=100.png" alt="" title="" width="500">
  <br>
  <em>Figure 3 –  Variation du bruit dynamique du filtre QEst = 100 </em>
</p>

<p align="center">
  <img src="Q3-QEst=200.png" alt="" title="" width="500">
  <br>
  <em>Figure 4 –  Variation du bruit dynamique du filtre QEst = 200 </em>
</p>

On observe qu’une élévation du bruit dynamique engendre une plus grande variabilité des résultats. Autrement dit, l’erreur de calcul du filtre de Kalman ne croisse pas de façon marquée, les estimations deviennent davantage sujettes à des fluctuations d’une itération à l’autre, ce qui se traduit par des courbes présentant une moindre régularité et une apparence plus agitée. Dans les cas où QEst est trop grand, un bruit excessif conduit à une estimation instable, même si celle-ci reste possible.


## Q4

Ici on peut voir des simulations en faisant varier le bruit de mesure du filtre (REst):


<p align="center">
  <img src="Q4-REst=50.png" alt="" title="" width="500">
  <br>
  <em>Figure 5 – Variation du bruit de mesure du filtre REst = 50 </em>
</p>

<p align="center">
  <img src="Q4-REst=100.png" alt="" title="" width="500">
  <br>
  <em>Figure 6 –  Variation du bruit de mesure du filtre REst = 100 </em>
</p>

<p align="center">
  <img src="Q4-REst=200.png" alt="" title="" width="500">
  <br>
  <em>Figure 7 –  Variation du bruit de mesure du filtre REst = 200 </em>
</p>

On remarque que lorsque le bruit de mesure s’intensifie, la dispersion des observations devient plus importante, même si l’erreur globale reste pratiquement inchangée. Par ailleurs, l’augmentation de la covariance associée à la position se traduit par une expansion de l’ellipse de confiance, alors que la trajectoire générale conserve une forme globalement stable et peu affectée.


## Q5 

La variation du seuil de ré-échantillonnage (\theta_{\mathrm{eff}}) met clairement en évidence l’impact de ce paramètre sur le comportement du filtre particulaire. On peut voir ça dans la figure suivant.


<p align="center">
  <img src="Q5.png" alt="Histogramme" title="Variation de theta_eff" width="500">
  <br>
  <em>Figure 8 – Variation de theta_eff </em>
</p>


Pour des valeurs faibles ((0.0 \leq \theta_{\mathrm{eff}} \leq 0.1)), les histogrammes des poids sont très déséquilibrés : quelques particules ont des poids élevés, les autres sont presque nulles. Cela correspond à une **dégénérescence** du filtre, avec une forte perte de diversité et une baisse de (N_{\mathrm{eff}}).

Pour une valeur intermédiaire ((\theta_{\mathrm{eff}} \approx 0.2)), la distribution des poids devient plus équilibrée et (N_{\mathrm{eff}}) augmente. Le ré-échantillonnage intervient alors au bon moment, maintenant la diversité tout en limitant la concentration excessive.

Pour des valeurs élevées ((\theta_{\mathrm{eff}} \geq 0.4)), les poids deviennent presque uniformes et (N_{\mathrm{eff}}) atteint son maximum. Cela empêche la dégénérescence, mais un ré-échantillonnage trop fréquent peut réduire la diversité des particules.

Ainsi, un **seuil trop bas** favorise la dégénérescence, tandis qu’un **seuil trop haut** peut induire une sur-représentation prématurée. Un **compromis intermédiaire** permet de maintenir une bonne précision tout en préservant la diversité.


## Q6

Ici on peut voir une simulation d'un trou de mesures entre 250 et 350 secondes:

<p align="center">
  <img src="Q6.png" alt="Mesures entre 2500s et 3500s" title="Simulation d'un trou notValidCondition" width="500">
  <br>
  <em>Figure 11 – Simulation d'un trou notValidCondition = true </em>
</p>

Il apparaît clairement que, pendant l’interruption des mesures entre 2500 et 3000 secondes (en comparaison avec la simulation initiale où notValidCondition = false sur la figure Q2), on observe une augmentation de l’erreur ainsi que de la covariance, conséquence directe de l’absence de correction par le filtre de Kalman. Toutefois, dès que les mesures sont rétablies, le filtre reprend son fonctionnement normal, ajuste progressivement les estimations et parvient, après quelques itérations, à retrouver un état similaire à celui de la simulation complète.


## Q7


Ici sont présentées quelques simulations résultant de la variation de la fréquence de mesure

<p align="center">
  <img src="Q3-dt_meas=50.png" alt="" title="" width="500">
  <br>
  <em>Figure 2 – Variation de la fréquence de mesure dt_meas = 50 </em>
</p>

<p align="center">
  <img src="Q3-dt_meas=100.png" alt="" title="" width="500">
  <br>
  <em>Figure 3 –  Variation de la fréquence de mesure dt_meas = 100 </em>
</p>

<p align="center">
  <img src="Q3-dt_meas=200.png" alt="" title="" width="500">
  <br>
  <em>Figure 4 –  Variation de la fréquence de mesure dt_meas = 200 </em>
</p>

On peut voir que l’allongement de l’intervalle entre les mesures provoque une dégradation notable de la trajectoire, traduisant une diminution de la précision du calcul effectué par le filtre de Kalman. Cette dégradation se manifeste à travers plusieurs éléments, tels qu’une augmentation des erreurs sur les états du système, une hausse de l’écart-type et un élargissement de l’ellipse de covariance. Cette situation s’explique par le fait qu’un intervalle de mesure plus long entraîne davantage d’itérations entre deux observations successives, ce qui favorise l’accumulation des erreurs et, par conséquent, une détérioration des performances globales du filtre de Kalman.

Ici on peut voir des simulations en faisant varier le nombre d'amers sur la carte (nLandmarks):

<p align="center">
  <img src="Q7-n=4.png" alt="" title="" width="500">
  <br>
  <em>Figure 12 – nLandmarks = 4 </em>
</p>

<p align="center">
  <img src="Q7-n=100.png" alt="" title="" width="500">
  <br>
  <em>Figure 13 –  nLandmarks = 100 </em>
</p>

<p align="center">
  <img src="Q7-n=200.png" alt="" title="" width="500">
  <br>
  <em>Figure 14 – nLandmarks = 200 </em>
</p>


On constate qu’en accroissant le nombre d'amers dans l’environnement de simulation, les performances du filtre de Kalman s’améliorent légèrement : l’erreur, la covariance et la taille de l’ellipse diminuent, signe d’un fonctionnement plus précis de l’algorithme. Cette amélioration s’explique sans doute par une meilleure sélection des références, offrant un compromis optimal entre erreurs angulaires et erreurs de distance.


## Q8

Ici on peut voir des simulations où seulement les mesures de distance sont disponibles et aussi en faisant varier le nombre d'amers sur la carte (nLandmarks):

<p align="center">
  <img src="Q8-n=4.png" alt="" title="" width="500">
  <br>
  <em>Figure 15 – nLandmarks = 4 </em>
</p>

<p align="center">
  <img src="Q8-n=100.png" alt="" title="" width="500">
  <br>
  <em>Figure 16 –  nLandmarks = 100 </em>
</p>

<p align="center">
  <img src="Q8-n=200.png" alt="" title="" width="500">
  <br>
  <em>Figure 17 –  nLandmarks = 200 </em>
</p>

On observe que lorsque seules les mesures de distance sont utilisées pour le filtre de Kalman, les résultats deviennent plus fluctuants, avec une variance plus élevée pour tous les états et une trajectoire moins lisse. Cependant, le filtre reste précis, l’aire de l’ellipse ne montrant pas de différence significative par rapport au cas de référence. Ainsi, le nombre de références sur la carte n’a pas d’impact majeur sur le fonctionnement du filtre.


## Q9

Ici on peut voir des simulations où seulement les mesures de direction sont disponibles et aussi en faisant varier le nombre d'amers sur la carte (nLandmarks):

<p align="center">
  <img src="Q9-n=4.png" alt="" title="" width="500">
  <br>
  <em>Figure 18 – nLandmarks = 4 </em>
</p>

<p align="center">
  <img src="Q9-n=100.png" alt="" title="" width="500">
  <br>
  <em>Figure 19 –  nLandmarks = 100 </em>
</p>

<p align="center">
  <img src="Q9-n=200.png" alt="" title="" width="500">
  <br>
  <em>Figure 20 –  nLandmarks = 200 </em>
</p>


On constate que lorsque seules les mesures d’angle sont utilisées dans le filtre de Kalman, la trajectoire reste globalement stable. Contrairement au cas des distances, l’ajout de références supplémentaires améliore nettement la précision de la trajectoire, montrant que les mesures d’angle sont particulièrement sensibles à la présence de références dans la simulation.
