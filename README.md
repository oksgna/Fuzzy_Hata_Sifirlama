Üyelik fonksiyonlarının merkez değerlerini daraltmak 
Çıkarım yöntemi max-min yerine max-product yapmak
Berraklaştırma da ağırlık merkezi yerine LOM (Largest of Maximum) kullanmak
 
Yeni üyelik fonksiyonu değerleri:
e_NB = (-400, -250, -150)
e_NS = (-200, -75, -10)
e_Z  = (-20, 0, 20)
e_PS = (10, 75, 200)
e_PB = (150, 250, 400)
 
Yeni hata değişimi (de) değerleri
de_N = (-200, -80, 0)
de_Z = (-30, 0, 30)
de_P = (0, 80, 200)
 
Çıkarım yöntemi:
aggregated = np.maximum(aggregated, mu_out * fire)   # MAX–PRODUCT
 
Berraklaştırma Yöntemi:
u_out = u_disc[np.argmax(aggregated)]

Bu değişiklikler sonucu hata 0’a gider. 
