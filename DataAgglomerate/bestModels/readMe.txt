note sui modelli addestrati:
inserire:
	-NOME modello
	-Data
	-CLASSI su cui è stato addestrato | DATASET
	-PERFORMANCE (accuracy,..)
	-MEMORIA OCCUPATA (big,lite,micro)

-----------------------------------
best_model_1
DATASET: "DataAgglomerate". CLASSI: FRANCESCO, RICCARDO, ELIA, ELISA, GRECO
accuracy: ~1
memory big: --

-----------------------------------

best_model_2
15/5
DATASET: "DataAgglomerate". CLASSI: FRANCESCO, RICCARDO, ELIA, ELISA, GRECO, MARA, ANNICK
accuracy: 1
memory quantized:74312
memory lite: 73584
 q_model_2_1: input=8integer
 q_model_2_2: input=32 float


------------------------------------
best_model_3_CONV2_incomplete
DATASET: aggiunti dati Ludo. Esclusi questi file(per usarli nel test):
1_2_F
2_2_R
2_4_R
4_2_E
6_1_Mara
7_0_Annick
8_1_Ludo
8_6_Ludo
8_7_Ludo

poi metterli nel test che ho appena scritto in colab