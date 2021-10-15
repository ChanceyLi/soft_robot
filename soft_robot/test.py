import model

m = model.Model([0, 0, 0, 0, 0, 0], 0.1)
m.forward_matrix()
print(m.T_soft)
