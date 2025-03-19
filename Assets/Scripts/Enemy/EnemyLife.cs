using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class EnemyLife : MonoBehaviour
{
    public float maxHealth = 100f;
    private float currentHealth;
    public float healthThreshold = 0.5f;

    private void Start()
    {
        currentHealth = maxHealth;
    }

    private void Update()
    {
        if (Input.GetKeyDown(KeyCode.X))
        {
            TakeDamage(currentHealth / 2 + 1);
        }
    }

    public void TakeDamage(float damageAmount)
    {
        currentHealth = Mathf.Max(0, currentHealth -= damageAmount);

        // Si la vida es 0, morir
        if (currentHealth <= 0)
        {
            Die();
        }
    }

    private void Die()
    {
        Debug.Log("Enemy died!");
        Destroy(gameObject);
    }

    public bool IsLowHealth()
    {
        return currentHealth <= maxHealth * healthThreshold;
    }
}
