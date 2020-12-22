# Simulation du picar S2 sous blender.

Mettre les fichiers blender dans le dossier "blender"

##  **pour faire un script qui affecte une scène blender:**

###  **setup de vscode**

le python se trouve ici (pour 2.90 sur windows 10):
* "C:\Program Files\Blender Foundation\Blender 2.90\2.90\python\bin"

Pour faire fonctionner blender avec le script, en ligne de commande:
* "C:\Program Files\Blender Foundation\Blender 2.90\2.90\python\bin\python" -m pip install matplotlib --user
* "C:\Program Files\Blender Foundation\Blender 2.90\2.90\python\bin\python" -m pip install threading --user
* <span style="color:red">C'est incomplet, il y a un bug avec le pip de blender. Il n'arrive pas à trouver les modules après. Pour corriger ça j'ai copier les fichier à la main dans le dossier de module du python à blender. Je me souviens pu exactement du path où il le met.</span>
* Installer l'extension blender Development:

  **Name**: Blender Development
  
  **Id**: jacqueslucke.blender-development

  **Description**: Tools to simplify Blender development.

  **Version**: 0.0.12

  **Publisher**: Jacques Lucke

  VS Marketplace Link: https://marketplace.visualstudio.com/items?itemName=JacquesLucke.blender-development
  
  **note**: * <span style="color:red">L'extension est buggé et supporte pas nos caractère français. Donc pas de suprise si é se transforme en @ quelque chose</span>

* Démarer blender: ctrl+shift+p: Blender start
  
 ![blender Start](markdown/startBlender.png) 

 ![Image of Yaktocat](markdown/debugger.png)

* Allez dans le fichier python désiré puis ctrl+shift+p: blender run script



Projet S5 UDES, robot suiveur de ligne, université de sherbrooke
