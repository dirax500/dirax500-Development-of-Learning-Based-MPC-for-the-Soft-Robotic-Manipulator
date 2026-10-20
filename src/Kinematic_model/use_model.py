from softri_inverse_model import *

df, rest = build_dataset("dataset/Data_Col.csv",
                         "dataset/softri_ml_dataset_20261008_111120.csv")
pose = df[["z", "roll", "pitch"]].values
rate = pose_rate(pose)
model = InverseModel(seeds=5).fit(pose, rate, df[["m1", "m2", "m3"]].values)

# desired pose: z = -40 mm, roll = 5 deg, pitch = -8 deg, held still
motors = model.predict_rel(np.array([[-40, 5, -8]]), np.zeros((1, 3))) + rest
print("motor positions (deg):", motors)