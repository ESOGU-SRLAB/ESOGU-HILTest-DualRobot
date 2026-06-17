import json

# Girdi ve çıktı dosyalarının adları
input_filename = 'real_trajectories.json'
output_filename = 'sim_trajectories.json'

try:
    # Kaynak JSON dosyasını aç ve yükle
    with open(input_filename, 'r') as f:
        trajectories_data = json.load(f)

    # Veri içindeki her bir yörüngeyi işle
    for trajectory_name, trajectory_info in trajectories_data.items():
        original_joint_names = trajectory_info.get('joint_names', [])
        sim_joint_names = []
        
        # Her bir eklem adı için yeni dönüşüm kuralını uygula
        for name in original_joint_names:
            if name == 'ur10e_base_to_robot_mount':
                # Özel kuralı uygula
                sim_joint_names.append('sim_ur10e_base_to_robot_mount')
            else:
                # Genel kuralı uygula ("sim_" öneki ekle)
                sim_joint_names.append(f"sim_{name}")
        
        # Eski eklem adları listesini yenisiyle değiştir
        trajectory_info['joint_names'] = sim_joint_names
        print(f"'{trajectory_name}' için eklem adları yeni kurala göre güncellendi.")

    # Değiştirilmiş veriyi yeni dosyaya kaydet
    with open(output_filename, 'w') as f:
        json.dump(trajectories_data, f, indent=2)
    
    print(f"\nDönüşüm tamamlandı. Yeni dosya '{output_filename}' adıyla oluşturuldu.")

except FileNotFoundError:
    print(f"Hata: Girdi dosyası '{input_filename}' bulunamadı. Lütfen betiğin doğru dizinde olduğundan emin olun.")
except json.JSONDecodeError:
    print(f"Hata: '{input_filename}' dosyası geçerli bir JSON formatında değil.")
except Exception as e:
    print(f"Beklenmeyen bir hata oluştu: {e}")
